"""Trajectory planning for foot swing using Bezier curves."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from .bezier import Bezier


@dataclass
class TrajectoryPlanner:
    """Generates foot-swing trajectories via Bezier interpolation.

    Example control-point setups (from original Trajectory.py):

    points_set_1 = [[-0.2, 0.105, -0.7], [-0.1, 0.105, -0.42],
                     [0.1, 0.105, -0.42],  [0.2, 0.105, -0.7]]
        → Mid-step swing, lateral = 0.105 (crotch height)

    points_set_2 = [[0, 0.105, -0.8], [0, 0.105, -0.69],
                     [0, 0.305, -0.68], [0, 0.305, -0.68]]
        → Step up with lateral shift

    points_set_3 = [[0, 0.105, -0.8], [0, 0.105, -0.69],
                     [0.142, 0.18, -0.68], [0.142, 0.18, -0.68]]
        → Step forward + up + lateral
    """

    control_points: np.ndarray
    """(N, 3) control points for the XYZ foot trajectory."""

    yaw_control: np.ndarray | None = None
    """Optional (M, 1) yaw trajectory. If None, use constant yaw."""

    dt: float = 0.01
    """Time step for trajectory discretisation."""

    yaw: float = 0.0
    """Constant yaw angle (used if yaw_control is None)."""

    def __post_init__(self):
        self.control_points = np.asarray(self.control_points, dtype=float)

    @property
    def t_values(self) -> np.ndarray:
        return np.arange(0, 1.0, self.dt)

    def generate(self) -> tuple[np.ndarray, np.ndarray]:
        """Compute the trajectory.

        Returns:
            xyz: (M, 3) foot positions in meters
            yaw: (M, 1) yaw angles in radians
        """
        xyz = Bezier.curve(self.t_values, self.control_points)
        if self.yaw_control is not None:
            yaw_arr = Bezier.curve(self.t_values, np.asarray(self.yaw_control))
        else:
            yaw_arr = np.full((len(xyz), 1), self.yaw)
        return xyz, yaw_arr

    @classmethod
    def step(cls,
             start_xyz: list[float],
             end_xyz: list[float],
             apex_height: float = 0.05,
             lateral_offset: float = 0.0,
             dt: float = 0.01) -> "TrajectoryPlanner":
        """Convenience: create a single step trajectory.

        4 control points: start → mid (apex) → end, with optional
        lateral offset for swing-out.
        """
        sx, sy, sz = start_xyz
        ex, ey, ez = end_xyz
        mid_x = (sx + ex) / 2.0
        mid_y = (sy + ey) / 2.0 + lateral_offset
        mid_z = (sz + ez) / 2.0 + apex_height  # lift foot
        mid2_z = mid_z
        ctrl = np.array([
            [sx, sy, sz],
            [mid_x, mid_y, mid_z],
            [mid_x, mid_y + lateral_offset, mid_z],
            [ex, ey, ez],
        ])
        return cls(control_points=ctrl, dt=dt)
