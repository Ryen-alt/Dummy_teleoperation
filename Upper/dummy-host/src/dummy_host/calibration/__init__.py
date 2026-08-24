"""Camera and robot calibration primitives.

The modules in this package deliberately keep the numeric core independent of
ROS and SciPy so the same tools run in the lightweight dummy-host environment.
OpenCV is imported only by image/board operations.
"""

from .geometry import invert_transform, matrix_to_quaternion_xyzw
from .urdf import UrdfKinematics

__all__ = ["UrdfKinematics", "invert_transform", "matrix_to_quaternion_xyzw"]
