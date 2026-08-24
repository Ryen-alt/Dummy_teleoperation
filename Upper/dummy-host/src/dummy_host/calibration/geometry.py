from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np


class GeometryError(ValueError):
    pass


def normalize_vector(value: np.ndarray, *, name: str = "vector") -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise GeometryError(f"{name} must be a finite 3-vector")
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        raise GeometryError(f"{name} must be non-zero")
    return vector / norm


def project_rotation(rotation: np.ndarray) -> np.ndarray:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.isfinite(value).all():
        raise GeometryError("rotation must be a finite 3x3 matrix")
    left, _, right = np.linalg.svd(value)
    projected = left @ right
    if np.linalg.det(projected) < 0:
        left[:, -1] *= -1
        projected = left @ right
    return projected


def axis_angle_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = normalize_vector(axis, name="rotation axis")
    if not np.isfinite(angle_rad):
        raise GeometryError("rotation angle must be finite")
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    sine = math.sin(float(angle_rad))
    cosine = math.cos(float(angle_rad))
    return np.eye(3) + sine * skew + (1.0 - cosine) * (skew @ skew)


def rpy_rotation(rpy_rad: np.ndarray) -> np.ndarray:
    rpy = np.asarray(rpy_rad, dtype=np.float64)
    if rpy.shape != (3,) or not np.isfinite(rpy).all():
        raise GeometryError("rpy must be a finite 3-vector")
    roll, pitch, yaw = rpy
    return (
        axis_angle_rotation(np.array([0.0, 0.0, 1.0]), yaw)
        @ axis_angle_rotation(np.array([0.0, 1.0, 0.0]), pitch)
        @ axis_angle_rotation(np.array([1.0, 0.0, 0.0]), roll)
    )


def make_transform(
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    if rotation is not None:
        result[:3, :3] = project_rotation(rotation)
    if translation is not None:
        value = np.asarray(translation, dtype=np.float64)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise GeometryError("translation must be a finite 3-vector")
        result[:3, 3] = value
    return result


def validate_transform(transform: np.ndarray, *, name: str = "transform") -> np.ndarray:
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise GeometryError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise GeometryError(f"{name} has an invalid homogeneous row")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise GeometryError(f"{name} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise GeometryError(f"{name} rotation determinant is not +1")
    return value


def invert_transform(transform: np.ndarray) -> np.ndarray:
    value = validate_transform(transform)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -(value[:3, :3].T @ value[:3, 3])
    return result


def rotation_angle_rad(rotation: np.ndarray) -> float:
    value = project_rotation(rotation)
    cosine = float(np.clip((np.trace(value) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    value = project_rotation(rotation)
    angle = rotation_angle_rad(value)
    if angle < 1e-10:
        return np.zeros(3, dtype=np.float64)
    if math.pi - angle < 1e-6:
        eigenvalues, eigenvectors = np.linalg.eig(value)
        index = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, index])
        axis = normalize_vector(axis)
    else:
        axis = np.array(
            [
                value[2, 1] - value[1, 2],
                value[0, 2] - value[2, 0],
                value[1, 0] - value[0, 1],
            ]
        ) / (2.0 * math.sin(angle))
    return axis * angle


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = project_rotation(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion *= -1
    return quaternion


def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    value = np.asarray(quaternion, dtype=np.float64)
    if value.shape != (4,) or not np.isfinite(value).all():
        raise GeometryError("quaternion must be a finite xyzw 4-vector")
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        raise GeometryError("quaternion must be non-zero")
    x, y, z, w = value / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def average_transforms(transforms: Iterable[np.ndarray]) -> np.ndarray:
    values = [validate_transform(value) for value in transforms]
    if not values:
        raise GeometryError("at least one transform is required")
    rotation_sum = sum((value[:3, :3] for value in values), np.zeros((3, 3)))
    translation = np.mean([value[:3, 3] for value in values], axis=0)
    return make_transform(project_rotation(rotation_sum), translation)


def transform_error(reference: np.ndarray, estimate: np.ndarray) -> tuple[float, float]:
    delta = invert_transform(reference) @ validate_transform(estimate)
    translation_mm = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
    rotation_deg = math.degrees(rotation_angle_rad(delta[:3, :3]))
    return translation_mm, rotation_deg


def matrix_as_list(transform: np.ndarray) -> list[list[float]]:
    return validate_transform(transform).tolist()
