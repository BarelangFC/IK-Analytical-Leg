#!/usr/bin/env python3
"""Generate demo GIF of IK-Analytical-Leg walking trajectory."""

import math
import sys
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add parent to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ik_analytical_leg import (
    ik, fk, solve, Bezier, TrajectoryPlanner, LegParams, DEFAULT_PARAMS,
)
from ik_analytical_leg.kinematics import fk_full

# --- Trajectory: a single step ---
# Standing → lift foot → step forward → place down
ctrl = np.array([
    [0.0,   0.105, -0.652],
    [0.05,  0.105, -0.50],
    [0.12,  0.105, -0.45],
    [0.12,  0.14,  -0.45],
    [0.18,  0.14,  -0.50],
    [0.25,  0.105, -0.70],
    [0.30,  0.105, -0.75],
])

t_vals = np.linspace(0, 1, 60)
traj_xyz = Bezier.curve(t_vals, ctrl)
traj_yaw = np.zeros(len(t_vals))

# --- Compute joint angles for each trajectory point ---
joints_list = []
frames_list = []
for i in range(len(t_vals)):
    j = ik(traj_xyz[i, 0], traj_xyz[i, 1], traj_xyz[i, 2], yaw=traj_yaw[i])
    joints_list.append(j)
    frames_list.append(fk_full(j))

# --- Set up figure ---
fig = plt.figure(figsize=(10, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1], height_ratios=[1, 0.6],
                       hspace=0.3, wspace=0.3)
ax3d = fig.add_subplot(gs[0, :], projection="3d")
ax_joints = fig.add_subplot(gs[1, 0])
ax_info = fig.add_subplot(gs[1, 1])
ax_info.axis("off")

params = DEFAULT_PARAMS

# Pre-compute axes limits
all_pts = []
for f in frames_list:
    for name in ["base", "hip", "knee_up", "knee_down", "ankle", "sole"]:
        all_pts.append(f[name][:3, 3])
all_pts = np.array(all_pts)
margin = 0.05
xmin, xmax = all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin
ymin, ymax = all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin
zmin, zmax = all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin

FRAME_NAMES = ["base", "hip_yaw", "hip_roll", "hip", "knee_up", "knee_down",
               "ankle_pitch", "ankle", "sole"]
COLORS = ["#333", "#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00",
          "#ffff33", "#a65628", "#f781bf"]
LINK_PAIRS = [("base", "hip_yaw"), ("hip_yaw", "hip_roll"), ("hip_roll", "hip"),
              ("hip", "knee_up"), ("knee_up", "knee_down"),
              ("knee_down", "ankle_pitch"), ("ankle_pitch", "ankle"),
              ("ankle", "sole")]

# Info text
frame_idx_text = ax_info.text(0.05, 0.9, "", transform=ax_info.transAxes,
                               fontsize=12, fontfamily="monospace")
joint_text = ax_info.text(0.05, 0.5, "", transform=ax_info.transAxes,
                          fontsize=10, fontfamily="monospace")

joint_labels = ["qHY", "qHR", "qHP", "qAP", "qAR"]
joint_history = {label: [] for label in joint_labels}
joint_t = []


def init():
    ax3d.clear()
    ax_joints.clear()
    ax3d.set_xlim(xmin, xmax)
    ax3d.set_ylim(ymin, ymax)
    ax3d.set_zlim(zmin, zmax)
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("IK-Analytical-Leg — Demo")
    ax3d.view_init(elev=25, azim=-60)
    ax_joints.set_xlabel("Trajectory step")
    ax_joints.set_ylabel("Joint angle (deg)")
    ax_joints.set_ylim(-90, 90)
    ax_joints.axhline(0, color="#888", linewidth=0.5)
    ax_joints.legend(loc="upper right", fontsize=8)
    frame_idx_text.set_text("")
    joint_text.set_text("")
    return []


def update(frame):
    ax3d.clear()
    ax3d.set_xlim(xmin, xmax)
    ax3d.set_ylim(ymin, ymax)
    ax3d.set_zlim(zmin, zmax)
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title(f"IK-Analytical-Leg — Step trajectory (frame {frame}/{len(t_vals)})")
    ax3d.view_init(elev=25, azim=-60)

    f = frames_list[frame]
    # Draw links
    for src, dst in LINK_PAIRS:
        p_src = f[src][:3, 3]
        p_dst = f[dst][:3, 3]
        ax3d.plot([p_src[0], p_dst[0]], [p_src[1], p_dst[1]], [p_src[2], p_dst[2]],
                  "k-", linewidth=3, alpha=0.8)
    # Draw frames
    for name in ["base", "hip_yaw", "hip", "knee_up", "knee_down", "ankle_pitch",
                  "ankle", "sole"]:
        if name in f:
            T = f[name]
            scale = 0.03
            for c, axis in zip(["r", "g", "b"], [0, 1, 2]):
                end = T[:3, 3] + T[:3, :3] @ np.array([scale if i == axis else 0 for i in range(3)])
                ax3d.plot([T[0, 3], end[0]], [T[1, 3], end[1]], [T[2, 3], end[2]],
                          c, linewidth=1.5, alpha=0.7)
    # Plot full trajectory
    ax3d.plot(traj_xyz[:, 0], traj_xyz[:, 1], traj_xyz[:, 2], "b--", alpha=0.4, linewidth=1)
    ax3d.scatter(*traj_xyz[frame], color="tab:blue", s=40, zorder=5)
    ax3d.scatter(*ctrl.T, color="tab:orange", marker="x", s=30, alpha=0.6)

    # Joint angle plot
    ax_joints.clear()
    ax_joints.set_xlabel("Trajectory step")
    ax_joints.set_ylabel("Joint angle (deg)")
    ax_joints.set_ylim(-90, 90)
    ax_joints.axhline(0, color="#888", linewidth=0.5)
    ax_joints.set_title("Joint angles")
    steps = np.arange(len(t_vals))

    j = joints_list[frame]
    j_deg = [math.degrees(v) for v in j]
    for name, val in zip(joint_labels, j_deg):
        joint_history[name].append(val)
    joint_t.append(frame)

    colors_j = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00"]
    for idx, name in enumerate(joint_labels):
        data = joint_history[name]
        x_vals = joint_t[-len(data):]
        ax_joints.plot(x_vals, data, color=colors_j[idx], label=name, linewidth=1.5)

    # Only add legend once after enough data
    if len(joint_t) > 1:
        ax_joints.legend(loc="upper right", fontsize=7)

    # Info text
    frame_idx_text.set_text(f"Frame: {frame}/{len(t_vals)}")
    info_lines = [
        "Joint Angles (deg)",
        f"  qHY: {j_deg[0]:+7.2f}",
        f"  qHR: {j_deg[1]:+7.2f}",
        f"  qHP: {j_deg[2]:+7.2f}",
        f"  qAP: {j_deg[3]:+7.2f}",
        f"  qAR: {j_deg[4]:+7.2f}",
        "",
        f"Foot: ({traj_xyz[frame,0]:+.3f}, {traj_xyz[frame,1]:+.3f}, {traj_xyz[frame,2]:+.3f})"
    ]
    joint_text.set_text("\n".join(info_lines))

    return []


print("Generating demo animation...")
anim = FuncAnimation(fig, update, frames=len(t_vals), init_func=init,
                     interval=50, blit=False)

out_path = os.path.join(os.path.dirname(__file__), "..", "demo.gif")
print(f"Saving to {out_path} ...")
anim.save(out_path, writer="pillow", fps=20, dpi=100)
print(f"Done! GIF saved to {out_path}")
