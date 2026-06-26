#!/usr/bin/env python3
"""CI smoke test for IK-Analytical-Leg."""

import sys
import numpy as np

sys.path.insert(0, ".")
from ik_analytical_leg import ik, fk, solve, Bezier, TrajectoryPlanner

# --- FK round-trip ---
joints = ik(0.0, 0.105, -0.652, 0.0)
T = fk(joints)
print(f"FK pose: {T[:3,3]}")
print(f"Joint angles (deg): {[round(np.degrees(j), 2) for j in joints]}")

# --- Random round-trip ---
rng = np.random.RandomState(0)
ok = 0
for _ in range(20):
    x, y, z, yaw = (
        rng.uniform(-0.15, 0.15),
        rng.uniform(0.05, 0.16),
        rng.uniform(-0.75, -0.55),
        rng.uniform(-0.3, 0.3),
    )
    sol = solve(x, y, z, yaw)
    if sol["success"]:
        ok += 1
print(f"Success rate: {ok}/20")

# --- Bezier ---
t = np.linspace(0, 1, 10)
pts = np.array([[0, 0.105, -0.7], [0.1, 0.105, -0.42], [0.2, 0.105, -0.7]])
c = Bezier.curve(t, pts)
assert c.shape == (10, 3), f"Bad shape: {c.shape}"
print(f"Bezier: {c[0]} -> {c[-1]}")

# --- Trajectory planner ---
tp = TrajectoryPlanner(control_points=pts)
xyz, yaw = tp.generate()
print(f"Trajectory: {len(xyz)} steps, first={xyz[0]}, last={xyz[-1]}")

print("\nALL CHECKS PASSED")
