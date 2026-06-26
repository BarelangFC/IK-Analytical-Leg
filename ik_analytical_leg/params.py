"""Robot leg kinematic parameters for Barelang FC humanoid."""

from dataclasses import dataclass


@dataclass(frozen=False)
class LegParams:
    """Kinematic parameters of the humanoid leg (left leg).

    All lengths in meters, angles in radians.

    Kinematic chain:
      base → hip_yaw (Rz) → hip_roll (Rx) → hip_pitch (Ry) → [hip]
      → knee_up (Ry) → knee_down_link → ankle_pitch (Ry) → ankle_roll (Rx)
      → sole

    The leg uses a parallel mechanism where:
      knee_up angle = hip_pitch angle
      knee_down angle = ankle_pitch angle (opposite sign)
    """

    # — Link lengths —
    crotch_to_hip: float = 0.105    # lateral distance: base → hip_yaw
    upper_hip: float = 0.0625       # vertical: hip_yaw → hip_roll/pitch
    hip_to_knee: float = 0.275      # upper leg (thigh)
    knee_to_ankle: float = 0.275    # lower leg (shin)
    ankle_to_sole: float = 0.0455   # foot height
    up_knee_to_down_knee: float = 0.16  # knee parallel-mechanism offset

    # — Joint offsets (FK) —
    hip_dx: float = 0.05179         # hip roll x-offset
    hip_dz: float = -0.02           # hip base z-offset
    ankle_dx: float = -0.05189      # sole x-offset from ankle

    # — Derived (readability aliases) —
    @property
    def A(self) -> float:
        return self.hip_to_knee

    @property
    def B(self) -> float:
        return self.knee_to_ankle

    @property
    def D(self) -> float:
        return self.crotch_to_hip


# Default instance matching original Trajectory.py
DEFAULT_PARAMS = LegParams()
