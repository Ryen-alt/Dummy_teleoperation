from __future__ import annotations

import argparse
import copy
import fcntl
import json
import math
import struct
import threading
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from .gamepad import ConfiguredGamepadProtocolAdapter, GamepadSource
from .cartesian_teleop import CartesianGamepadMapper
from .teleop import GamepadMapper, KeyboardMapper, TeleopCommand, TeleopError, TeleopProfile


class InputDeviceError(TeleopError):
    pass


_GAMEPAD_LINK_DIRS = (Path("/dev/input/by-id"), Path("/dev/input/by-path"))

# Linux include/uapi/linux/input.h:
#   #define EVIOCSCLOCKID _IOW('E', 0xa0, int)
#
# python-evdev 1.9.x exposes the event file descriptor but does not wrap this
# ioctl.  Configure it directly so event.sec/event.usec share the host
# CLOCK_MONOTONIC timebase used by the rest of the control pipeline.
_EVIOCSCLOCKID = 0x400445A0


@dataclass(frozen=True)
class InputDeviceInfo:
    path: str
    name: str
    physical_path: str
    has_keys: bool
    absolute_axes: tuple[str, ...]
    vendor_id: int | None = None
    product_id: int | None = None


@dataclass(frozen=True)
class InputSnapshot:
    """One immutable evdev state published at a SYN_REPORT boundary."""

    pressed: frozenset[str]
    axes: tuple[tuple[str, float], ...]
    event_ns: int
    snapshot_ns: int
    sync_lost: bool

    @property
    def axis_values(self) -> dict[str, float]:
        return dict(self.axes)


def _load_evdev() -> tuple[Any, Any, Callable[[], list[str]]]:
    try:
        from evdev import InputDevice, ecodes, list_devices
    except ImportError as exc:
        raise InputDeviceError("install dummy-host[teleop] to use Linux evdev inputs") from exc
    return InputDevice, ecodes, list_devices


def _set_monotonic_event_clock(device: Any) -> None:
    try:
        descriptor = int(device.fd)
        fcntl.ioctl(
            descriptor,
            _EVIOCSCLOCKID,
            struct.pack("i", int(time.CLOCK_MONOTONIC)),
        )
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        raise InputDeviceError(
            f"cannot configure CLOCK_MONOTONIC event timestamps: {exc}"
        ) from exc


def _code_name(ecodes: Any, event_type: int, code: int) -> str:
    value = ecodes.bytype[event_type].get(code, str(code))
    if isinstance(value, list):
        return str(value[0])
    return str(value)


def list_input_devices() -> list[InputDeviceInfo]:
    InputDevice, ecodes, list_devices = _load_evdev()
    output: list[InputDeviceInfo] = []
    for path in sorted(list_devices()):
        device = None
        try:
            device = InputDevice(path)
            capabilities = device.capabilities(absinfo=False)
            axes = tuple(
                sorted(_code_name(ecodes, ecodes.EV_ABS, int(code)) for code in capabilities.get(ecodes.EV_ABS, []))
            )
            output.append(
                InputDeviceInfo(
                    path=path,
                    name=device.name or "",
                    physical_path=device.phys or "",
                    has_keys=bool(capabilities.get(ecodes.EV_KEY)),
                    absolute_axes=axes,
                    vendor_id=getattr(device.info, "vendor", None),
                    product_id=getattr(device.info, "product", None),
                )
            )
        except OSError:
            continue
        finally:
            if device is not None:
                device.close()
    return output


class _EvdevDevice:
    def __init__(self, path: str) -> None:
        InputDevice, self.ecodes, _ = _load_evdev()
        try:
            self.device = InputDevice(path)
        except OSError as exc:
            raise InputDeviceError(f"cannot open input device {path}: {exc}") from exc
        self.path = path
        self.last_error: str | None = None
        self._closed = False
        self._lock = threading.Lock()
        self._active_keys: set[int] = set()
        self._axis_values: dict[int, Any] = {}
        self._last_event_ns = time.monotonic_ns()
        self._sync_lost = False
        event_reader = getattr(self.device, "read_loop", None)
        if callable(event_reader):
            try:
                _set_monotonic_event_clock(self.device)
            except InputDeviceError:
                self.device.close()
                self._closed = True
                raise
        try:
            self._active_keys = set(int(code) for code in self.device.active_keys())
        except (OSError, SystemError) as exc:
            self.last_error = str(exc)
        self._pending_active_keys = set(self._active_keys)
        self._pending_axis_values: dict[int, Any] = {}
        self._reader: threading.Thread | None = None
        if callable(event_reader):
            self._reader = threading.Thread(
                target=self._event_loop,
                name="dummy-evdev-events",
                daemon=True,
            )
            self._reader.start()

    def resolve(self, name: str, *, expected_type: int) -> int:
        try:
            code = int(self.ecodes.ecodes[name])
        except (KeyError, TypeError, ValueError) as exc:
            raise InputDeviceError(f"unknown evdev code {name}") from exc
        capabilities = self.device.capabilities(absinfo=False)
        if code not in capabilities.get(expected_type, []):
            raise InputDeviceError(f"{self.path} does not expose {name}")
        return code

    def pressed(self, configured: dict[str, int]) -> set[str]:
        with self._lock:
            if self.last_error is not None:
                raise InputDeviceError(f"input device disconnected: {self.last_error}")
            active = set(self._active_keys)
        return {name for name, code in configured.items() if code in active}

    def axis(self, code: int) -> float:
        with self._lock:
            if self.last_error is not None:
                raise InputDeviceError(f"input device disconnected: {self.last_error}")
            info = self._axis_values.get(code)
        if info is None:
            try:
                info = self.device.absinfo(code)
            except (OSError, SystemError) as exc:
                self.last_error = str(exc)
                raise InputDeviceError(f"input device disconnected: {exc}") from exc
            with self._lock:
                self._axis_values[code] = info
                self._pending_axis_values.setdefault(code, info)
        return self._normalize_axis(info)

    @staticmethod
    def _normalize_axis(info: Any) -> float:
        minimum = float(info.min)
        maximum = float(info.max)
        value = float(info.value)
        if not all(math.isfinite(item) for item in (minimum, maximum, value)):
            raise InputDeviceError("input device returned a non-finite axis value")
        if maximum <= minimum:
            raise InputDeviceError("input device axis range is invalid")
        centre = (minimum + maximum) * 0.5
        half_range = (maximum - minimum) * 0.5
        return max(-1.0, min(1.0, (value - centre) / half_range))

    def snapshot(
        self,
        configured_keys: Mapping[str, int],
        configured_axes: Mapping[str, int],
        snapshot_ns: int,
    ) -> InputSnapshot:
        """Read keys, axes, and timing from one published evdev generation."""

        with self._lock:
            if self.last_error is not None:
                raise InputDeviceError(f"input device disconnected: {self.last_error}")
            axes: list[tuple[str, float]] = []
            try:
                for name, code in configured_axes.items():
                    info = self._axis_values.get(code)
                    if info is None:
                        info = self.device.absinfo(code)
                        self._axis_values[code] = info
                        self._pending_axis_values.setdefault(code, info)
                    axes.append((name, self._normalize_axis(info)))
            except (OSError, SystemError) as exc:
                self.last_error = str(exc)
                raise InputDeviceError(f"input device disconnected: {exc}") from exc
            result = InputSnapshot(
                pressed=frozenset(
                    name
                    for name, code in configured_keys.items()
                    if code in self._active_keys
                ),
                axes=tuple(sorted(axes)),
                event_ns=self._last_event_ns,
                snapshot_ns=snapshot_ns,
                sync_lost=self._sync_lost,
            )
            self._sync_lost = False
            return result

    def event_metadata(self) -> tuple[int, bool]:
        with self._lock:
            event_ns = self._last_event_ns
            sync_lost = self._sync_lost
            self._sync_lost = False
        return event_ns, sync_lost

    def _event_loop(self) -> None:
        try:
            for event in self.device.read_loop():
                if self._closed:
                    return
                if hasattr(event, "sec") and hasattr(event, "usec"):
                    event_ns = int(event.sec) * 1_000_000_000 + int(event.usec) * 1_000
                else:
                    event_ns = int(event.timestamp() * 1e9)
                with self._lock:
                    if event.type == self.ecodes.EV_KEY:
                        if event.value:
                            self._pending_active_keys.add(int(event.code))
                        else:
                            self._pending_active_keys.discard(int(event.code))
                    elif event.type == self.ecodes.EV_ABS:
                        current = self._pending_axis_values.get(int(event.code))
                        if current is None:
                            current = self._axis_values.get(int(event.code))
                        if current is None:
                            current = self.device.absinfo(event.code)
                        if hasattr(current, "_replace"):
                            current = current._replace(value=event.value)
                        else:
                            current = copy.copy(current)
                            current.value = event.value
                        self._pending_axis_values[int(event.code)] = current
                    elif (
                        event.type == self.ecodes.EV_SYN
                        and event.code == self.ecodes.SYN_REPORT
                    ):
                        self._active_keys = set(self._pending_active_keys)
                        self._axis_values = dict(self._pending_axis_values)
                        self._last_event_ns = event_ns
                    elif (
                        event.type == self.ecodes.EV_SYN
                        and event.code == self.ecodes.SYN_DROPPED
                    ):
                        self._sync_lost = True
                        resynced_keys = set(
                            int(code) for code in self.device.active_keys()
                        )
                        resynced_axes = {
                            code: self.device.absinfo(code)
                            for code in tuple(self._axis_values)
                        }
                        self._active_keys = resynced_keys
                        self._pending_active_keys = set(resynced_keys)
                        self._axis_values = resynced_axes
                        self._pending_axis_values = dict(resynced_axes)
                        self._last_event_ns = event_ns
        except (OSError, SystemError) as exc:
            if not self._closed:
                with self._lock:
                    self.last_error = str(exc)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self.device.close()
            if self._reader is not None:
                self._reader.join(timeout=1.0)


class EvdevKeyboardSource:
    def __init__(self, path: str, profile: TeleopProfile) -> None:
        self._device = _EvdevDevice(path)
        self._mapper = KeyboardMapper(profile)
        mapping = profile.keyboard
        names = {
            *mapping.joint_positive,
            *mapping.joint_negative,
            mapping.gripper_open,
            mapping.gripper_close,
            mapping.deadman,
            mapping.hold,
            mapping.estop,
            *mapping.episode_buttons.values(),
        }
        try:
            self._keys = {
                name: self._device.resolve(name, expected_type=self._device.ecodes.EV_KEY)
                for name in names
            }
        except BaseException:
            self._device.close()
            raise

    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        try:
            snapshotter = getattr(self._device, "snapshot", None)
            if callable(snapshotter):
                snapshot = snapshotter(self._keys, {}, now_ns)
                pressed = set(snapshot.pressed)
                event_ns = snapshot.event_ns
                sync_lost = snapshot.sync_lost
            else:
                metadata = getattr(self._device, "event_metadata", None)
                event_ns, sync_lost = (
                    metadata() if callable(metadata) else (now_ns, False)
                )
                pressed = self._device.pressed(self._keys)
            command = self._mapper.map(pressed, now_ns)
            return replace(
                command,
                event_ns=event_ns,
                raw={**command.raw, "input_sync_lost": sync_lost},
            )
        except InputDeviceError as exc:
            return _with_raw_error(
                self._mapper.map(set(), now_ns, connected=False), str(exc)
            )

    def close(self) -> None:
        self._device.close()


class EvdevGamepadSource:
    def __init__(
        self,
        path: str,
        profile: TeleopProfile,
        *,
        teleop_mode: str = "joint",
    ) -> None:
        self._device = _EvdevDevice(path)
        if teleop_mode == "joint":
            self._mapper = GamepadMapper(profile)
        elif teleop_mode == "cartesian":
            self._mapper = CartesianGamepadMapper(profile)
        else:
            self._device.close()
            raise InputDeviceError("teleop_mode must be 'joint' or 'cartesian'")
        mapping = profile.gamepad
        protocol = mapping.protocol
        if protocol.transport != "evdev":
            self._device.close()
            raise InputDeviceError(
                f"EvdevGamepadSource cannot open transport {protocol.transport!r}"
            )
        self._adapter = ConfiguredGamepadProtocolAdapter(protocol)
        logical_button_names = {
            mapping.gripper_open,
            mapping.gripper_close,
            mapping.deadman,
            mapping.hold,
            *mapping.estop_chord,
            *mapping.episode_buttons.values(),
        }
        if teleop_mode == "joint":
            logical_axis_names = {binding.axis for binding in mapping.joint_axes}
        else:
            assert profile.cartesian is not None
            logical_axis_names = set(profile.cartesian.axis_names)
        physical_button_names = {protocol.buttons[name] for name in logical_button_names}
        physical_axis_names = {protocol.axes[name].code for name in logical_axis_names}
        try:
            self._buttons = {
                name: self._device.resolve(name, expected_type=self._device.ecodes.EV_KEY)
                for name in physical_button_names
            }
            self._axes = {
                name: self._device.resolve(
                    name, expected_type=self._device.ecodes.EV_ABS
                )
                for name in physical_axis_names
            }
        except BaseException:
            self._device.close()
            raise

    def poll(self, now_ns: int | None = None) -> TeleopCommand:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        try:
            snapshotter = getattr(self._device, "snapshot", None)
            if callable(snapshotter):
                snapshot = snapshotter(self._buttons, self._axes, now_ns)
                physical_pressed = set(snapshot.pressed)
                physical_axes = snapshot.axis_values
                event_ns = snapshot.event_ns
                sync_lost = snapshot.sync_lost
            else:
                physical_pressed = self._device.pressed(self._buttons)
                physical_axes = {
                    name: self._device.axis(code) for name, code in self._axes.items()
                }
                metadata = getattr(self._device, "event_metadata", None)
                event_ns, sync_lost = (
                    metadata() if callable(metadata) else (now_ns, False)
                )
            state = self._adapter.decode(
                physical_axes,
                physical_pressed,
                now_ns,
                raw={"device_path": self._device.path},
            )
            command = self._mapper.map_state(state)
            return replace(
                command,
                event_ns=event_ns,
                raw={**command.raw, "input_sync_lost": sync_lost},
            )
        except InputDeviceError as exc:
            state = self._adapter.decode({}, set(), now_ns, connected=False)
            command = self._mapper.map_state(state)
            return _with_raw_error(command, str(exc))

    def close(self) -> None:
        self._device.close()


GamepadSourceFactory = Callable[[str, TeleopProfile], GamepadSource]


def resolve_gamepad_endpoint(endpoint: str) -> str:
    """Resolve ``auto`` to one stable event-joystick link."""
    if endpoint != "auto":
        return endpoint

    # De-duplicate by the final event node because by-id and by-path can both
    # point at the same physical controller. Prefer by-id by search order.
    links_by_target: dict[Path, Path] = {}
    for directory in _GAMEPAD_LINK_DIRS:
        if not directory.is_dir():
            continue
        for link in sorted(directory.glob("*event-joystick")):
            try:
                target = link.resolve(strict=True)
            except OSError:
                continue
            links_by_target.setdefault(target, link)

    links = sorted(links_by_target.values())
    if not links:
        raise InputDeviceError(
            "--device auto found no live /dev/input/by-id/ or by-path/ "
            "*-event-joystick link"
        )
    if len(links) > 1:
        choices = ", ".join(str(link) for link in links)
        raise InputDeviceError(
            f"--device auto found multiple gamepads ({choices}); pass one link explicitly"
        )
    return str(links[0])


def create_gamepad_source(
    endpoint: str,
    profile: TeleopProfile,
    *,
    factories: Mapping[str, GamepadSourceFactory] | None = None,
    teleop_mode: str = "joint",
) -> GamepadSource:
    """Create a transport source; applications may register vendor protocols."""
    builders: dict[str, GamepadSourceFactory] = {"evdev": EvdevGamepadSource}
    if factories:
        builders.update(factories)
    transport = profile.gamepad.protocol.transport
    try:
        factory = builders[transport]
    except KeyError as exc:
        raise InputDeviceError(
            f"no gamepad source factory registered for transport {transport!r}"
        ) from exc
    if transport == "evdev":
        endpoint = resolve_gamepad_endpoint(endpoint)
    if factory is EvdevGamepadSource:
        source = factory(endpoint, profile, teleop_mode=teleop_mode)
    else:
        if teleop_mode != "joint":
            raise InputDeviceError(
                "custom gamepad transports must provide their own Cartesian-mode source"
            )
        source = factory(endpoint, profile)
    if not isinstance(source, GamepadSource):
        raise InputDeviceError(f"factory for {transport!r} did not return a GamepadSource")
    return source


def _with_raw_error(command: TeleopCommand, error: str) -> TeleopCommand:
    return TeleopCommand(
        monotonic_ns=command.monotonic_ns,
        source=command.source,
        joint_velocity_rad_s=np.zeros(6, dtype=np.float32),
        gripper_velocity_per_s=0.0,
        deadman=False,
        hold_requested=True,
        estop_requested=False,
        episode_event=None,
        connected=False,
        raw={"error": error},
        teleop_mode=command.teleop_mode,
        cartesian_twist=np.zeros(6, dtype=np.float32),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="List Linux evdev keyboard/gamepad candidates")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args()
    devices = list_input_devices()
    if args.json:
        print(json.dumps([asdict(item) for item in devices], indent=2, ensure_ascii=False))
        return
    for item in devices:
        axes = ",".join(item.absolute_axes) if item.absolute_axes else "-"
        print(f"{item.path}\t{item.name}\tkeys={item.has_keys}\tabs={axes}")


if __name__ == "__main__":
    main()
