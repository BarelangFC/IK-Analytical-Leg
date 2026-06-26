"""Bezier curve evaluation (n-point, arbitrary dimension)."""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


class Bezier:
    """De Casteljau Bézier curve evaluation."""

    @staticmethod
    def two_points(t: float, P1: np.ndarray, P2: np.ndarray) -> np.ndarray:
        """Linear interpolation between two points."""
        return (1.0 - t) * P1 + t * P2

    @staticmethod
    def points(t: float, points: list[np.ndarray]) -> list[np.ndarray]:
        """One De Casteljau reduction step."""
        return [Bezier.two_points(t, points[i], points[i + 1])
                for i in range(len(points) - 1)]

    @staticmethod
    def point(t: float, points: list[np.ndarray]) -> np.ndarray:
        """Single point on the Bézier curve at parameter t ∈ [0, 1]."""
        pts = points[:]
        while len(pts) > 1:
            pts = Bezier.points(t, pts)
        return pts[0]

    @staticmethod
    def curve(t_values: np.ndarray, points: np.ndarray) -> np.ndarray:
        """Evaluate full curve at multiple parameter values.

        Args:
            t_values: 1-D array of parameter values (typically np.arange(0, 1, dt))
            points: (N, D) array of control points

        Returns:
            (len(t_values), D) array of curve points
        """
        pts_list = [points[i] for i in range(points.shape[0])]
        out = np.zeros((len(t_values), points.shape[1]))
        for i, t in enumerate(t_values):
            out[i] = Bezier.point(t, pts_list)
        return out

    @staticmethod
    def trajectory(t_values: np.ndarray,
                   points: Sequence[np.ndarray | list[float]],
                   yaw: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """Bezier curve for foot trajectory + yaw spline.

        Args:
            t_values: parameter sweep (e.g. np.arange(0, 1, 0.01))
            points: control points (each shape (3,) for xyz)
            yaw: constant yaw angle (or pass yaw_control separately)

        Returns:
            (xyz_curve (M, 3), yaw_curve (M, 1))
        """
        pts_np = np.array([np.asarray(p, dtype=float) for p in points])
        curve_xyz = Bezier.curve(t_values, pts_np)
        curve_yaw = np.full((len(t_values), 1), yaw)
        return curve_xyz, curve_yaw
