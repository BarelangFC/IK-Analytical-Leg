"""Forward and inverse kinematics for 7-DOF humanoid leg with parallel mechanism.

Joint order: [qHipYaw, qHipRoll, qHipPitch, qAnklePitch, qAnkleRoll]
  - qHipYaw = Z rotation at hip base
  - qHipRoll = X rotation at hip (adduction/abduction)
  - qHipPitch = Y rotation at hip (flexion/extension)
  - qAnklePitch = Y rotation at ankle
  - qAnkleRoll = X rotation at ankle (inversion/eversion)

Parallel mechanism: knee_up = hip_pitch, knee_down = ankle_pitch (coupled).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .params import LegParams, DEFAULT_PARAMS


# ======================================================================
#  Homogeneous transform helpers
# ======================================================================

def RX(alpha: float) -> np.ndarray:
    """3x3 rotation about X-axis."""
    c, s = math.cos(alpha), math.sin(alpha)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def RY(delta: float) -> np.ndarray:
    """3x3 rotation about Y-axis."""
    c, s = math.cos(delta), math.sin(delta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def RZ(theta: float) -> np.ndarray:
    """3x3 rotation about Z-axis."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def TF(rot_axis: str | None = None, q: float = 0.0,
       dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> np.ndarray:
    """4x4 homogeneous transform: rotation + translation."""
    if rot_axis == "x":
        R = RX(q)
    elif rot_axis == "y":
        R = RY(q)
    elif rot_axis == "z":
        R = RZ(q)
    else:
        R = np.eye(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [dx, dy, dz]
    return T


def transform_inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4×4 homogeneous transform [R|t] ⟶ [R^T | -R^T t]."""
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def rotation_matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Extract (roll, pitch, yaw) from rotation matrix (ZYX Euler).

    Returns: (roll, pitch, yaw) — radians, matching PyKDL's GetEulerZYX
    convention where the returned order is (Z, Y, X).
    """
    # singularity checks
    if R[2, 0] < 1.0:
        if R[2, 0] > -1.0:
            pitch = math.asin(-R[2, 0])
            roll = math.atan2(R[2, 1], R[2, 2])
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:  # R[2,0] == -1 → pitch = +90°
            pitch = math.pi / 2.0
            roll = math.atan2(-R[1, 2], R[1, 1])
            yaw = 0.0
    else:  # R[2,0] == +1 → pitch = -90°
        pitch = -math.pi / 2.0
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """3×3 rotation from ZYX Euler angles: R = RZ(yaw) RY(pitch) RX(roll)."""
    return RZ(yaw) @ RY(pitch) @ RX(roll)


def compose_frame(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """4×4 frame from 3×3 rotation + 3-element translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ======================================================================
#  Error metric
# ======================================================================

def pose_error(T_target: np.ndarray, T_result: np.ndarray) -> tuple[float, list[float]]:
    """Pose error between two 4×4 frames.

    Returns (scalar_error, [dx, dy, dz, roll, pitch, yaw]).
    """
    T_diff = transform_inverse(T_target) @ T_result
    dx, dy, dz = T_diff[:3, 3]
    roll, pitch, yaw = rotation_matrix_to_rpy(T_diff[:3, :3])
    error = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2 + roll ** 2 + pitch ** 2 + yaw ** 2)
    return error, [dx, dy, dz, roll, pitch, yaw]


# ======================================================================
#  Forward Kinematics
# ======================================================================

def fk(joints: list[float] | np.ndarray,
       params: LegParams = DEFAULT_PARAMS) -> np.ndarray:
    """Foot sole pose from 5 joint angles.

    Args:
        joints: [qHipYaw, qHipRoll, qHipPitch, qAnklePitch, qAnkleRoll]
        params: leg kinematic parameters

    Returns:
        4×4 homogeneous transform of the sole frame (relative to base).
    """
    qHY, qHR, qHP, qAP, qAR = joints

    T = np.eye(4)

    # base → hip_yaw  (Rz, dy, dz)
    T = T @ TF("z", qHY, dy=params.D, dz=params.hip_dz)
    # hip_yaw → hip_roll (Rx, dx, dz)
    T = T @ TF("x", qHR, dx=params.hip_dx, dz=-params.upper_hip)
    # hip_roll → hip_pitch (Ry) — [hip frame]
    T = T @ TF("y", qHP)
    # hip → knee (Ry, dz) — parallel: knee_up = qHP
    T = T @ TF("y", -qHP, dz=-params.A)
    # knee → knee_down_link (Ry, dz) — parallel: knee_down = -qAP
    T = T @ TF("y", -qAP, dz=-params.up_knee_to_down_knee)
    # knee_down → ankle_pitch (Ry, dz) — parallel: ankle = +qAP
    T = T @ TF("y", qAP, dz=-params.B)
    # ankle_pitch → ankle_roll (Rx)
    T = T @ TF("x", -qHR)
    # ankle → sole (dx, dz)
    T = T @ TF(dx=params.ankle_dx, dz=-params.ankle_to_sole)

    return T


def fk_full(joints: list[float] | np.ndarray,
            params: LegParams = DEFAULT_PARAMS) -> dict[str, np.ndarray]:
    """Forward kinematics returning every intermediate frame.

    Args:
        joints: [qHipYaw, qHipRoll, qHipPitch, qAnklePitch, qAnkleRoll]

    Returns:
        dict mapping frame names to 4×4 transforms.
    """
    qHY, qHR, qHP, qAP, qAR = joints

    T = np.eye(4)
    frames: dict[str, np.ndarray] = {"base": T.copy()}

    T = T @ TF("z", qHY, dy=params.D, dz=params.hip_dz)
    frames["hip_yaw"] = T.copy()
    T = T @ TF("x", qHR, dx=params.hip_dx, dz=-params.upper_hip)
    frames["hip_roll"] = T.copy()
    T = T @ TF("y", qHP)
    frames["hip"] = T.copy()
    T = T @ TF("y", -qHP, dz=-params.A)
    frames["knee_up"] = T.copy()
    T = T @ TF("y", qAP, dz=-params.up_knee_to_down_knee)
    frames["knee_down"] = T.copy()
    T = T @ TF("y", -qAP, dz=-params.B)
    frames["ankle_pitch"] = T.copy()
    T = T @ TF("x", -qHR)
    frames["ankle"] = T.copy()
    T = T @ TF(dx=params.ankle_dx, dz=-params.ankle_to_sole)
    frames["sole"] = T.copy()

    return frames


# ======================================================================
#  Inverse Kinematics
# ======================================================================

def ik(x: float, y: float, z: float, yaw: float = 0.0,
       params: LegParams = DEFAULT_PARAMS) -> np.ndarray:
    """Analytical IK: foot position → leg joint angles.

    Args:
        x, y, z: target foot sole position (meters, relative to base frame).
            y is lateral (crotch direction), positive = outwards.
        yaw: foot yaw angle (radians).

    Returns:
        ndarray [qHipYaw, qHipRoll, qHipPitch, qAnklePitch, qAnkleRoll]
        or zeros if target is unreachable (NaN).
    """
    # 1. Target relative to hip frame
    xh = x
    yh = y - params.D      # subtract crotch-to-hip lateral offset
    zh = (z + params.upper_hip + params.ankle_to_sole
          + params.up_knee_to_down_knee)

    # 2. Apply yaw rotation to get planar hip coordinates
    xa = xh
    ya_1 = xa * math.tan(yaw)
    xb_1 = xa / math.cos(yaw)
    beta = math.pi / 2 - yaw
    ya_2 = yh - ya_1
    yb = ya_2 * math.sin(beta)
    xb_2 = yb / math.tan(beta)
    xh = xb_1 + xb_2
    yh = yb

    # 3. Distance from hip to target
    C = math.sqrt(xh ** 2 + yh ** 2 + zh ** 2)

    # 4. Hip/ankle roll (parallel)
    zh_roll = zh - params.up_knee_to_down_knee
    qHipRoll = math.atan2(yh, abs(zh_roll))
    # qAnkleRoll = -qHipRoll (applied later via parallel coupling in FK)

    # 5. Knee angle (law of cosines)
    A, B = params.A, params.B
    cos_q4 = (A ** 2 + B ** 2 - C ** 2) / (2 * A * B)
    cos_q4 = max(-1.0, min(1.0, cos_q4))  # clamp for numerical stability
    q4 = -math.acos(cos_q4)
    qKnee = q4 + math.pi  # convert to knee bend angle

    # 6. Hip pitch
    q3 = qKnee / 2
    qx = math.asin(max(-1.0, min(1.0, xh / C)))
    qHipPitch = -(q3 + qx)

    # 7. Ankle pitch
    qz = math.sqrt(yh ** 2 + zh ** 2)
    qAnklePitch = (math.pi - math.atan2(xh, math.copysign(1.0, zh) * qz)) - q3
    if qAnklePitch > math.radians(120):
        qAnklePitch -= math.radians(360)

    qHipYaw = yaw
    qAnkleRoll = -qHipRoll

    joints = np.array([qHipYaw, qHipRoll, qHipPitch, qAnklePitch, qAnkleRoll])

    # NaN guard — unreachable target
    if math.isnan(qHipPitch):
        return np.zeros(5)

    return joints


# ======================================================================
#  Convenience
# ======================================================================

def solve(x: float, y: float, z: float, yaw: float = 0.0,
          params: LegParams = DEFAULT_PARAMS) -> dict[str, Any]:
    """IK + FK + error in one call.

    Returns dict with keys: joints, T_solution, T_target, error, error_list, success.
    """
    joints = ik(x, y, z, yaw, params)
    T_sol = fk(joints, params)
    # Build target frame
    T_target = compose_frame(rpy_to_rotation(0.0, 0.0, yaw),
                             np.array([x, y, z]))
    err, err_list = pose_error(T_target, T_sol)
    return {
        "joints": joints,
        "T_solution": T_sol,
        "T_target": T_target,
        "error": err,
        "error_list": err_list,
        "success": err < 0.01,
    }
