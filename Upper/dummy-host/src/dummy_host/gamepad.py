from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Protocol, runtime_checkable


XBOX_AXES = (
    "left_x",
    "left_y",
    "right_x",
    "right_y",
    "left_trigger",
    "right_trigger",
    "dpad_x",
    "dpad_y",
)
XBOX_BUTTONS = (
    "a",
    "b",
    "x",
    "y",
    "lb",
    "rb",
    "view",
    "menu",
    "xbox",
    "left_stick",
    "right_stick",
)


class GamepadProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class PhysicalAxisBinding:
    code: str
    invert: bool = False

    def __post_init__(self) -> None:
        if not self.code:
            raise GamepadProtocolError("physical axis code must be non-empty")
        if not isinstance(self.invert, bool):
            raise GamepadProtocolError("physical axis invert must be boolean")


@dataclass(frozen=True)
class GamepadProtocolConfig:
    """Map transport-specific control codes onto stable logical controls."""

    protocol_id: str
    transport: str
    axes: Mapping[str, PhysicalAxisBinding]
    buttons: Mapping[str, str]

    def __post_init__(self) -> None:
        if not self.protocol_id or not self.transport:
            raise GamepadProtocolError("protocol_id and transport must be non-empty")
        axes = dict(self.axes)
        buttons = dict(self.buttons)
        if not axes or not buttons:
            raise GamepadProtocolError("gamepad protocol must define axes and buttons")
        if any(not name or not isinstance(binding, PhysicalAxisBinding) for name, binding in axes.items()):
            raise GamepadProtocolError("gamepad logical axes and bindings are invalid")
        if any(not name or not isinstance(code, str) or not code for name, code in buttons.items()):
            raise GamepadProtocolError("gamepad logical buttons and codes are invalid")
        if len({binding.code for binding in axes.values()}) != len(axes):
            raise GamepadProtocolError("physical axis codes must be unique")
        if len(set(buttons.values())) != len(buttons):
            raise GamepadProtocolError("physical button codes must be unique")
        object.__setattr__(self, "axes", MappingProxyType(axes))
        object.__setattr__(self, "buttons", MappingProxyType(buttons))


@dataclass(frozen=True)
class GamepadState:
    monotonic_ns: int
    axes: Mapping[str, float]
    pressed: frozenset[str]
    connected: bool = True
    protocol_id: str = "unknown"
    raw: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.monotonic_ns < 0 or not self.protocol_id:
            raise GamepadProtocolError("gamepad timestamp and protocol ID are invalid")
        axes = {name: float(value) for name, value in self.axes.items()}
        if any(not name or not math.isfinite(value) or not -1.0 <= value <= 1.0 for name, value in axes.items()):
            raise GamepadProtocolError("logical gamepad axes must be finite values in [-1, 1]")
        pressed = frozenset(self.pressed)
        if any(not isinstance(name, str) or not name for name in pressed):
            raise GamepadProtocolError("logical gamepad button names must be non-empty strings")
        object.__setattr__(self, "axes", MappingProxyType(axes))
        object.__setattr__(self, "pressed", pressed)
        object.__setattr__(self, "raw", MappingProxyType(dict(self.raw)))


@runtime_checkable
class GamepadProtocolAdapter(Protocol):
    @property
    def protocol_id(self) -> str: ...

    def decode(
        self,
        physical_axes: Mapping[str, float],
        physical_pressed: set[str],
        now_ns: int,
        *,
        connected: bool = True,
        raw: Mapping[str, object] | None = None,
    ) -> GamepadState: ...


class ConfiguredGamepadProtocolAdapter:
    """Data-driven adapter usable by evdev and custom communication transports."""

    def __init__(self, config: GamepadProtocolConfig) -> None:
        self.config = config

    @property
    def protocol_id(self) -> str:
        return self.config.protocol_id

    def decode(
        self,
        physical_axes: Mapping[str, float],
        physical_pressed: set[str],
        now_ns: int,
        *,
        connected: bool = True,
        raw: Mapping[str, object] | None = None,
    ) -> GamepadState:
        axes: dict[str, float] = {}
        for logical_name, binding in self.config.axes.items():
            value = float(physical_axes.get(binding.code, 0.0))
            if not math.isfinite(value):
                raise GamepadProtocolError(f"physical axis {binding.code} is not finite")
            value = max(-1.0, min(1.0, value))
            axes[logical_name] = -value if binding.invert else value
        pressed = frozenset(
            logical_name
            for logical_name, physical_code in self.config.buttons.items()
            if physical_code in physical_pressed
        )
        raw_payload = {
            "physical_axes": dict(sorted(physical_axes.items())),
            "physical_pressed": sorted(physical_pressed),
        }
        if raw:
            raw_payload.update(raw)
        return GamepadState(
            monotonic_ns=now_ns,
            axes=axes,
            pressed=pressed,
            connected=connected,
            protocol_id=self.protocol_id,
            raw=raw_payload,
        )


@runtime_checkable
class GamepadSource(Protocol):
    """Transport-neutral source for evdev, serial, Bluetooth or vendor SDK adapters."""

    def poll(self, now_ns: int | None = None): ...
    def close(self) -> None: ...
