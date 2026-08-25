from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from threading import Lock
from types import SimpleNamespace

import yaml
import pytest

import dummy_host.input_evdev as input_evdev
from dummy_host.apps.gamepad_mapping_test import _render, demo_results
from dummy_host.gamepad import ConfiguredGamepadProtocolAdapter, GamepadSource
from dummy_host.input_evdev import (
    EvdevGamepadSource,
    InputDeviceError,
    _EvdevDevice,
    create_gamepad_source,
)
from dummy_host.teleop import GamepadMapper, TeleopConfigError, load_teleop_profile


def _profile():
    return load_teleop_profile(Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml")


def test_evdev_background_reader_uses_kernel_time_and_marks_syn_dropped() -> None:
    ecodes = SimpleNamespace(
        EV_SYN=0,
        EV_KEY=1,
        EV_ABS=3,
        SYN_REPORT=0,
        SYN_DROPPED=3,
    )
    events = (
        SimpleNamespace(type=1, code=30, value=1, sec=1, usec=10),
        SimpleNamespace(type=0, code=0, value=0, sec=1, usec=20),
        SimpleNamespace(type=0, code=3, value=0, sec=1, usec=30),
    )

    class Device:
        def read_loop(self):
            return iter(events)

        def active_keys(self):
            return [31]

    device = _EvdevDevice.__new__(_EvdevDevice)
    device.device = Device()
    device.ecodes = ecodes
    device._closed = False
    device._lock = Lock()
    device._active_keys = set()
    device._axis_values = {}
    device._last_event_ns = 0
    device._sync_lost = False
    device.last_error = None

    device._event_loop()

    assert device._active_keys == {31}
    assert device.event_metadata() == (1_000_030_000, True)
    assert device.event_metadata() == (1_000_030_000, False)


def test_evdev_configures_monotonic_clock_through_linux_ioctl(monkeypatch) -> None:
    ioctl_calls: list[tuple[int, int, bytes]] = []

    class InputDevice:
        fd = 42

        def __init__(self, path: str) -> None:
            self.path = path
            self.closed = False

        def active_keys(self):
            return []

        def read_loop(self):
            return iter(())

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr(
        input_evdev,
        "_load_evdev",
        lambda: (InputDevice, SimpleNamespace(), lambda: []),
    )
    monkeypatch.setattr(
        input_evdev.fcntl,
        "ioctl",
        lambda fd, request, argument: ioctl_calls.append((fd, request, argument)),
    )

    device = _EvdevDevice("/dev/input/event42")
    device.close()

    assert len(ioctl_calls) == 1
    descriptor, request, argument = ioctl_calls[0]
    assert descriptor == 42
    assert request == input_evdev._EVIOCSCLOCKID
    assert input_evdev.struct.unpack("i", argument) == (input_evdev.time.CLOCK_MONOTONIC,)


def test_evdev_rejects_device_when_monotonic_clock_ioctl_fails(monkeypatch) -> None:
    opened: list[object] = []

    class InputDevice:
        fd = 43

        def __init__(self, path: str) -> None:
            self.path = path
            self.closed = False
            opened.append(self)

        def read_loop(self):
            return iter(())

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr(
        input_evdev,
        "_load_evdev",
        lambda: (InputDevice, SimpleNamespace(), lambda: []),
    )

    def fail_ioctl(fd, request, argument):
        raise OSError(22, "Invalid argument")

    monkeypatch.setattr(input_evdev.fcntl, "ioctl", fail_ioctl)

    with pytest.raises(InputDeviceError, match="cannot configure CLOCK_MONOTONIC"):
        _EvdevDevice("/dev/input/event43")
    assert opened and opened[0].closed


def test_flydigi_evdev_protocol_decodes_to_stable_logical_controls() -> None:
    profile = _profile()
    protocol = profile.gamepad.protocol
    assert protocol.protocol_id == "flydigi_vader5_linux_evdev_v1"
    assert protocol.transport == "evdev"
    adapter = ConfiguredGamepadProtocolAdapter(protocol)
    state = adapter.decode(
        {"ABS_X": 0.75, "ABS_Y": -0.5, "ABS_HAT0X": 1.0},
        {"BTN_TL", "BTN_NORTH"},
        1_000,
    )
    assert state.axes["left_x"] == 0.75
    assert state.axes["left_y"] == -0.5
    assert state.axes["dpad_x"] == 1.0
    assert state.pressed == frozenset({"lb", "x"})
    command = GamepadMapper(profile).map_state(state)
    assert command.deadman
    assert command.joint_velocity_rad_s[0] > 0
    assert command.joint_velocity_rad_s[1] > 0
    assert command.gripper_velocity_per_s < 0


def test_custom_evdev_codes_do_not_change_robot_operation_mapping(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    raw["gamepad"]["protocol"]["id"] = "vendor_pad_evdev_v1"
    raw["gamepad"]["protocol"]["axes"]["left_x"] = "ABS_THROTTLE"
    raw["gamepad"]["protocol"]["buttons"]["lb"] = "BTN_TRIGGER_HAPPY1"
    custom_path = tmp_path / "custom-pad.yaml"
    custom_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    custom = load_teleop_profile(custom_path)
    assert custom.gamepad.protocol.axes["left_x"].code == "ABS_THROTTLE"
    assert custom.gamepad.protocol.buttons["lb"] == "BTN_TRIGGER_HAPPY1"
    assert custom.gamepad.joint_axes == _profile().gamepad.joint_axes


def test_mapping_cannot_reference_a_control_missing_from_protocol(tmp_path) -> None:
    source = Path(__file__).parents[1] / "configs" / "teleop_inputs.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    del raw["gamepad"]["protocol"]["buttons"]["lb"]
    bad_path = tmp_path / "missing-deadman.yaml"
    bad_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(TeleopConfigError, match="missing from protocol"):
        load_teleop_profile(bad_path)


def test_custom_transport_factory_is_pluggable() -> None:
    class VendorSource:
        def poll(self, now_ns=None):
            return object()

        def close(self) -> None:
            pass

    profile = _profile()
    vendor_protocol = replace(profile.gamepad.protocol, transport="vendor_serial")
    vendor_profile = replace(
        profile,
        gamepad=replace(profile.gamepad, protocol=vendor_protocol),
    )
    source = create_gamepad_source(
        "COM42",
        vendor_profile,
        factories={"vendor_serial": lambda endpoint, selected: VendorSource()},
    )
    assert isinstance(source, GamepadSource)


def test_evdev_flydigi_source_resolves_physical_codes_then_maps_logical_controls(
    monkeypatch, tmp_path
) -> None:
    class ECodes:
        EV_KEY = 1
        EV_ABS = 3

    class FakeEvdevDevice:
        def __init__(self, path: str) -> None:
            self.path = path
            self.ecodes = ECodes()
            self.closed = False
            self.resolved: set[str] = set()

        def resolve(self, name: str, *, expected_type: int):
            self.resolved.add(name)
            return name

        def pressed(self, configured):
            return {"BTN_TL", "BTN_NORTH"}

        def axis(self, code):
            return {"ABS_X": 0.8, "ABS_Y": -0.6}.get(code, 0.0)

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr("dummy_host.input_evdev._EvdevDevice", FakeEvdevDevice)
    event_node = tmp_path / "event42"
    event_node.touch()
    stable_link = tmp_path / "usb-test-event-joystick"
    stable_link.symlink_to(event_node)
    monkeypatch.setattr("dummy_host.input_evdev._GAMEPAD_LINK_DIRS", (tmp_path,))
    source = create_gamepad_source("auto", _profile())
    assert isinstance(source, EvdevGamepadSource)
    assert source._device.path == str(stable_link)
    command = source.poll(10_000)
    assert command.deadman
    assert command.joint_velocity_rad_s[0] > 0
    assert command.joint_velocity_rad_s[1] > 0
    assert command.gripper_velocity_per_s < 0
    assert command.raw["transport"]["device_path"] == str(stable_link)
    # Unused trigger/mode controls do not make a compatible controller fail startup.
    assert "ABS_Z" not in source._device.resolved
    assert "BTN_MODE" not in source._device.resolved
    source.close()
    assert source._device.closed


def test_evdev_unplug_system_error_is_reported_as_disconnect(monkeypatch) -> None:
    class UnpluggedInputDevice:
        def __init__(self, path: str) -> None:
            self.path = path

        def active_keys(self):
            raise SystemError(
                "<built-in function ioctl_EVIOCG_bits> returned NULL "
                "without setting an exception"
            )

        def close(self) -> None:
            pass

    monkeypatch.setattr(
        "dummy_host.input_evdev._load_evdev",
        lambda: (UnpluggedInputDevice, object(), lambda: []),
    )
    device = _EvdevDevice("/dev/input/event7")
    with pytest.raises(InputDeviceError, match="input device disconnected"):
        device.pressed({})
    assert device.last_error is not None


def test_virtual_gamepad_demo_maps_all_six_joints_without_robot(config) -> None:
    results = demo_results(_profile(), config)
    assert results[0].status == "DEADMAN_RELEASED/HOLD"
    assert results[1].command.joint_velocity_rad_s[0] > 0
    assert results[2].command.joint_velocity_rad_s[1] > 0
    assert results[3].command.joint_velocity_rad_s[2] < 0
    assert results[4].command.joint_velocity_rad_s[3] > 0
    assert results[5].command.joint_velocity_rad_s[4] > 0
    assert results[6].command.joint_velocity_rad_s[5] > 0
    for joint_index, result in enumerate(results[1:7]):
        assert result.steps == 20
        assert result.target_delta is not None
        assert abs(result.target_delta[joint_index]) > 0
        assert sum(value != 0 for value in result.target_delta[:6]) == 1
    assert results[7].command.gripper_velocity_per_s < 0
    assert results[7].steps == 10
    assert results[7].target_delta is not None
    assert results[7].target_delta[6] < 0
    assert results[8].command.gripper_velocity_per_s > 0
    assert results[8].steps == 10
    assert results[8].target_delta is not None
    assert results[8].target_delta[6] > 0
    assert results[9].status == "HOLD_REQUESTED"
    assert results[10].status == "ESTOP_REQUESTED"


def test_human_render_exposes_disconnected_state(config, capsys) -> None:
    result = demo_results(_profile(), config)[0]
    disconnected = replace(
        result,
        status="DISCONNECTED/HOLD",
        command=replace(result.command, connected=False),
    )
    _render(disconnected, json_output=False)
    output = capsys.readouterr().out
    assert "DISCONNECTED/HOLD" in output
    assert "connected=false" in output


def test_gamepad_episode_buttons_are_edge_triggered() -> None:
    mapper = GamepadMapper(_profile())
    first = mapper.map({}, {"y"}, 1_000)
    held = mapper.map({}, {"y"}, 2_000)
    released = mapper.map({}, set(), 3_000)
    success = mapper.map({}, {"a"}, 4_000)
    assert first.episode_event == "start"
    assert held.episode_event is None
    assert released.episode_event is None
    assert success.episode_event == "success"
