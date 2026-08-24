from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np

from .geometry import axis_angle_rotation, make_transform, normalize_vector, rpy_rotation


class UrdfError(ValueError):
    pass


def _vector(element: ET.Element | None, attribute: str, default: str) -> np.ndarray:
    text = default if element is None else element.attrib.get(attribute, default)
    try:
        value = np.asarray([float(part) for part in text.split()], dtype=np.float64)
    except ValueError as exc:
        raise UrdfError(f"invalid {attribute} vector {text!r}") from exc
    if value.shape != (3,) or not np.isfinite(value).all():
        raise UrdfError(f"{attribute} must contain three finite values")
    return value


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None

    def transform(self, position: float = 0.0) -> np.ndarray:
        origin = make_transform(rpy_rotation(self.origin_rpy), self.origin_xyz)
        if self.joint_type == "fixed":
            return origin
        if self.joint_type in {"revolute", "continuous"}:
            motion = make_transform(axis_angle_rotation(self.axis, position))
            return origin @ motion
        if self.joint_type == "prismatic":
            motion = make_transform(translation=self.axis * position)
            return origin @ motion
        raise UrdfError(f"unsupported joint type {self.joint_type!r}")


class UrdfKinematics:
    """Minimal, deterministic URDF tree FK for calibration tooling."""

    def __init__(self, path: str | Path, *, base_link: str = "base_link", tip_link: str = "tool0"):
        self.path = Path(path)
        try:
            root = ET.parse(self.path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise UrdfError(f"cannot load URDF {self.path}: {exc}") from exc
        links = {link.attrib.get("name", "") for link in root.findall("link")}
        if base_link not in links or tip_link not in links:
            raise UrdfError(f"URDF must define {base_link!r} and {tip_link!r}")
        incoming: dict[str, UrdfJoint] = {}
        all_joints: dict[str, UrdfJoint] = {}
        for element in root.findall("joint"):
            joint = self._parse_joint(element)
            if joint.name in all_joints:
                raise UrdfError(f"duplicate joint {joint.name!r}")
            if joint.child in incoming:
                raise UrdfError(f"link {joint.child!r} has multiple parent joints")
            all_joints[joint.name] = joint
            incoming[joint.child] = joint
        chain_reversed: list[UrdfJoint] = []
        link = tip_link
        visited: set[str] = set()
        while link != base_link:
            if link in visited:
                raise UrdfError("cycle found while resolving URDF chain")
            visited.add(link)
            joint = incoming.get(link)
            if joint is None:
                raise UrdfError(f"no chain from {base_link!r} to {tip_link!r}")
            chain_reversed.append(joint)
            link = joint.parent
        self.base_link = base_link
        self.tip_link = tip_link
        self.chain = tuple(reversed(chain_reversed))
        self.movable_joints = tuple(
            joint for joint in self.chain if joint.joint_type != "fixed"
        )
        self._validate_tool0_contract()

    @staticmethod
    def _parse_joint(element: ET.Element) -> UrdfJoint:
        name = element.attrib.get("name", "").strip()
        joint_type = element.attrib.get("type", "").strip()
        parent_element = element.find("parent")
        child_element = element.find("child")
        if not name or not joint_type or parent_element is None or child_element is None:
            raise UrdfError("every joint needs name, type, parent and child")
        parent = parent_element.attrib.get("link", "").strip()
        child = child_element.attrib.get("link", "").strip()
        if not parent or not child:
            raise UrdfError(f"joint {name!r} has an empty parent or child")
        origin = element.find("origin")
        axis_element = element.find("axis")
        axis = _vector(axis_element, "xyz", "1 0 0")
        if joint_type != "fixed":
            axis = normalize_vector(axis, name=f"joint {name} axis")
        limit = element.find("limit")
        lower = upper = None
        if limit is not None:
            try:
                lower = float(limit.attrib["lower"]) if "lower" in limit.attrib else None
                upper = float(limit.attrib["upper"]) if "upper" in limit.attrib else None
            except ValueError as exc:
                raise UrdfError(f"joint {name!r} has an invalid limit") from exc
        return UrdfJoint(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=_vector(origin, "xyz", "0 0 0"),
            origin_rpy=_vector(origin, "rpy", "0 0 0"),
            axis=axis,
            lower=lower,
            upper=upper,
        )

    def _validate_tool0_contract(self) -> None:
        expected = tuple(f"joint_{index}" for index in range(1, 7))
        names = tuple(joint.name for joint in self.movable_joints)
        if names != expected:
            raise UrdfError(f"base_link->tool0 movable chain must be {expected}, got {names}")
        terminal = self.chain[-1]
        if (
            terminal.name != "tool0_fixed"
            or terminal.joint_type != "fixed"
            or terminal.parent != "hand_base"
            or terminal.child != "tool0"
            or not np.allclose(terminal.origin_xyz, 0.0)
            or not np.allclose(terminal.origin_rpy, 0.0)
        ):
            raise UrdfError(
                "tool0 must be an identity fixed frame whose parent is hand_base"
            )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.movable_joints)

    def validate_positions(self, positions_rad: np.ndarray) -> np.ndarray:
        values = np.asarray(positions_rad, dtype=np.float64)
        if values.shape != (len(self.movable_joints),) or not np.isfinite(values).all():
            raise UrdfError(
                f"joint positions must be {len(self.movable_joints)} finite radians"
            )
        for joint, value in zip(self.movable_joints, values, strict=True):
            if joint.lower is not None and value < joint.lower - 1e-9:
                raise UrdfError(f"{joint.name}={value:.6f} is below URDF limit {joint.lower:.6f}")
            if joint.upper is not None and value > joint.upper + 1e-9:
                raise UrdfError(f"{joint.name}={value:.6f} is above URDF limit {joint.upper:.6f}")
        return values

    def base_T_tool0(
        self,
        positions_rad: np.ndarray | Mapping[str, float],
        *,
        check_limits: bool = True,
    ) -> np.ndarray:
        if isinstance(positions_rad, Mapping):
            try:
                values = np.asarray(
                    [positions_rad[joint.name] for joint in self.movable_joints],
                    dtype=np.float64,
                )
            except KeyError as exc:
                raise UrdfError(f"missing joint position for {exc.args[0]!r}") from exc
        else:
            values = np.asarray(positions_rad, dtype=np.float64)
        if check_limits:
            values = self.validate_positions(values)
        elif values.shape != (len(self.movable_joints),) or not np.isfinite(values).all():
            raise UrdfError(
                f"joint positions must be {len(self.movable_joints)} finite radians"
            )
        position_by_name = dict(zip(self.joint_names, values, strict=True))
        result = np.eye(4, dtype=np.float64)
        for joint in self.chain:
            result = result @ joint.transform(position_by_name.get(joint.name, 0.0))
        return result

    def describe(self) -> dict[str, object]:
        return {
            "urdf": str(self.path.resolve()),
            "base_link": self.base_link,
            "tip_link": self.tip_link,
            "joint_names": list(self.joint_names),
            "chain": [joint.name for joint in self.chain],
            "tool0_parent": self.chain[-1].parent,
            "tool0_identity": True,
        }
