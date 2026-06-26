"""CLI entry points for IK-Analytical-Leg."""

import argparse
import math
import sys

import numpy as np

from .kinematics import ik, fk, solve, fk_full, LegParams, DEFAULT_PARAMS
from .bezier import Bezier


def _joint_str(j: np.ndarray) -> str:
    deg = [math.degrees(v) for v in j]
    return f"qHY={deg[0]:.3f}°  qHR={deg[1]:.3f}°  qHP={deg[2]:.3f}°  qAP={deg[3]:.3f}°  qAR={deg[4]:.3f}°"


def cmd_solve(args: argparse.Namespace) -> None:
    """IK + FK + error for a single foot pose."""
    result = solve(args.x, args.y, args.z, math.radians(args.yaw))
    print("Target foot pose:")
    print(f"  x={args.x:.4f}m  y={args.y:.4f}m  z={args.z:.4f}m  yaw={args.yaw:.2f}°")
    print()
    print(f"IK joints ({_joint_str(result['joints'])})")
    print()
    T = result["T_solution"]
    print(f"FK solution:  x={T[0,3]:.4f}m  y={T[1,3]:.4f}m  z={T[2,3]:.4f}m")
    print(f"Pose error:   {result['error']:.6f}")
    print(f"Success:      {'✓' if result['success'] else '✗'}")
    print(f"Error vector: [dx={result['error_list'][0]:.6f} "
          f"dy={result['error_list'][1]:.6f} "
          f"dz={result['error_list'][2]:.6f} "
          f"rx={result['error_list'][3]:.6f} "
          f"ry={result['error_list'][4]:.6f} "
          f"rz={result['error_list'][5]:.6f}]")


def cmd_trajectory(args: argparse.Namespace) -> None:
    """Compute foot trajectory from Bezier control points."""
    pts = parse_control_points(args.points)
    t_vals = np.arange(0, 1.0, args.dt)
    xyz = Bezier.curve(t_vals, pts)
    print(f"Trajectory ({len(xyz)} steps):")
    print(f"  From: ({xyz[0, 0]:.4f}, {xyz[0, 1]:.4f}, {xyz[0, 2]:.4f})")
    print(f"  To:   ({xyz[-1, 0]:.4f}, {xyz[-1, 1]:.4f}, {xyz[-1, 2]:.4f})")
    for i in range(0, len(xyz), max(1, len(xyz) // 10)):
        j = ik(xyz[i, 0], xyz[i, 1], xyz[i, 2], yaw=args.yaw)
        print(f"  t={t_vals[i]:.3f}  pos=({xyz[i, 0]:.4f}, {xyz[i, 1]:.4f}, {xyz[i, 2]:.4f})  "
              f"joints=({_joint_str(j)})")


def parse_control_points(raw: str) -> np.ndarray:
    """Parse control points from CLI arg.

    Format: "x1,y1,z1; x2,y2,z2; ..."
    """
    pts = []
    for group in raw.split(";"):
        group = group.strip()
        if not group:
            continue
        vals = [float(v.strip()) for v in group.split(",")]
        if len(vals) != 3:
            raise ValueError(f"Each point needs 3 values (x,y,z), got: {group}")
        pts.append(vals)
    if len(pts) < 2:
        raise ValueError("Need at least 2 control points")
    return np.array(pts)


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="ik-leg",
        description="IK-Analytical-Leg: Analytical IK/FK for humanoid robot leg",
    )
    sub = parser.add_subparsers(dest="command", help="Sub-command")

    # --- solve ---
    p_solve = sub.add_parser("solve", help="Solve IK/FK for a single foot pose")
    p_solve.add_argument("x", type=float, help="Foot x position (m)")
    p_solve.add_argument("y", type=float, help="Foot y position (m)")
    p_solve.add_argument("z", type=float, help="Foot z position (m)")
    p_solve.add_argument("--yaw", type=float, default=0.0, help="Foot yaw angle (degrees)")
    p_solve.set_defaults(func=cmd_solve)

    # --- trajectory ---
    p_traj = sub.add_parser("trajectory", help="Compute joint trajectory for Bezier foot path")
    p_traj.add_argument("points", type=str,
                        help="Control points: \"x1,y1,z1; x2,y2,z2; ...\"")
    p_traj.add_argument("--yaw", type=float, default=0.0, help="Constant foot yaw (degrees)")
    p_traj.add_argument("--dt", type=float, default=0.05, help="Time step (default 0.05)")
    p_traj.set_defaults(func=cmd_trajectory)

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        sys.exit(1)
    args.func(args)


if __name__ == "__main__":
    main()
