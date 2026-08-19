from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from .gamepad import ConfiguredGamepadProtocolAdapter, GamepadSource
from .teleop import GamepadMapper, KeyboardMapper, TeleopCommand, TeleopError, TeleopProfile


class InputDeviceError(TeleopError):
    pass


_GAMEPAD_LINK_DIRS = (Path("/dev/input/by-id"), Path("/dev/input/by-path"))


@dataclass(frozen=True)
class InputDeviceInfo:
    path: str
    name: str
    physical_path: str
    has_keys: bool
    absolute_axes: tuple[str, ...]
    vendor_id: int | None = None
    product_id: int | None = None


def _load_evdev() -> tuple[Any, Any, Callable[[], list[str]]]:
    try:
        from evdev import InputDevice, ecodes, list_devices
    except ImportError as exc:
        raise InputDeviceError("install dummy-host[teleop] to use Linux evdev inputs") from exc
    return InputDevice, ecodes, list_devices


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
        # python-evdev normally raises OSError after an unplug, but some
        # EVIOCGKEY ioctl failures surface as a bare SystemError instead.
        try:
            active = set(int(code) for code in self.device.active_keys())
        except (OSError, SystemError) as exc:
            self.last_error = str(exc)
            raise InputDeviceError(f"input device disconnected: {exc}") from exc
        return {name for name, code in configured.items() if code in active}

    def axis(self, code: int) -> float:
        try:
            info = self.device.absinfo(code)
        except (OSError, SystemError) as exc:
            self.last_error = str(exc)
            raise InputDeviceError(f"input device disconnected: {exc}") from exc
        minimum = float(info.min)
        maximum = float(info.max)
        if not all(math.isfinite(value) for value in (minimum, maximum, float(info.value))):
            raise InputDeviceError("input device returned a non-finite axis value")
        if maximum <= minimum:
            raise InputDeviceError("input device axis range is invalid")
        centre = (minimum + maximum) * 0.5
        half_range = (maximum - minimum) * 0.5
        return max(-1.0, min(1.0, (float(info.value) - centre) / half_range))

    def close(self) -> None:
        if not self._closed:
            self.device.close()
            self._closed = True


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
            return self._mapper.map(self._device.pressed(self._keys), now_ns)
        except InputDeviceError as exc:
            return _with_raw_error(
                self._mapper.map(set(), now_ns, connected=False), str(exc)
            )

    def close(self) -> None:
        self._device.close()


class EvdevGamepadSource:
    def __init__(self, path: str, profile: TeleopProfile) -> None:
        self._device = _EvdevDevice(path)
        self._mapper = GamepadMapper(profile)
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
        logical_axis_names = {binding.axis for binding in mapping.joint_axes}
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
            physical_pressed = self._device.pressed(self._buttons)
            physical_axes = {name: self._device.axis(code) for name, code in self._axes.items()}
            state = self._adapter.decode(
                physical_axes,
                physical_pressed,
                now_ns,
                raw={"device_path": self._device.path},
            )
            return self._mapper.map_state(state)
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
