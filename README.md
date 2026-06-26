# IK-Analytical-Leg

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![NumPy](https://img.shields.io/badge/dep-numpy-013243.svg)](https://numpy.org)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![CI](https://github.com/BarelangFC/IK-Analytical-Leg/actions/workflows/ci.yml/badge.svg)](https://github.com/BarelangFC/IK-Analytical-Leg/actions/workflows/ci.yml)

**Pure NumPy Analytical IK/FK solver for a 7-DOF humanoid robot leg.**

![Demo GIF](demo.gif)

## Overview

IK-Analytical-Leg provides an analytical closed-form inverse kinematics solution
for a humanoid robot leg with 7 degrees of freedom:

| Joint | Axis | Description |
|-------|------|-------------|
| qHipYaw | Z | Hip rotation (yaw) |
| qHipRoll | X | Hip adduction/abduction (roll) |
| qHipPitch | Y | Hip flexion/extension (pitch) |
| qAnklePitch | Y | Ankle flexion/extension (pitch) |
| qAnkleRoll | X | Ankle adduction/abduction (roll) |

The knee uses a **parallel mechanism**: the knee bend angle is split equally
between the hip pitch and ankle pitch joints, matching the physical 4-bar linkage
on Barelang FC humanoid legs.

### Leg chain

```
base → hip_yaw (Z) → hip_roll (X) → hip → knee_up → knee_down → ankle_pitch (Y) → ankle_roll (X) → sole
```

### Dependencies

- Python ≥ 3.10
- NumPy ≥ 1.21
- _(optional)_ matplotlib + pillow for GIF generation

## Install

```bash
pip install git+https://github.com/BarelangFC/IK-Analytical-Leg.git
```

Or for development (with demo tools):

```bash
git clone git@github.com:BarelangFC/IK-Analytical-Leg.git
cd IK-Analytical-Leg
pip install -e ".[demo]"
```

## Usage

### CLI

```bash
# Solve IK for a single foot pose
ik-leg solve 0 0.105 -0.652 --yaw 0

# Compute joint trajectory from Bezier control points
ik-leg trajectory "0,0.105,-0.7; 0.1,0.105,-0.42; 0.2,0.105,-0.7"
```

### Python API

```python
from ik_analytical_leg import ik, fk, solve, Bezier

# Single pose IK
joints = ik(x=0.0, y=0.105, z=-0.652, yaw=0.0)
# joints = [qHY, qHR, qHP, qAP, qAR]

# IK + FK verification
result = solve(x=0.12, y=0.105, z=-0.65, yaw=0.05)
if result["success"]:
    print(f"Joints (deg): {[round(j, 2) for j in result['joints']]}")
    print(f"Error: {result['error']:.6f}")

# Bezier foot trajectory
import numpy as np
t_vals = np.linspace(0, 1, 50)
ctrl = np.array([[0, 0.105, -0.7], [0.1, 0.105, -0.42], [0.2, 0.105, -0.7]])
xyz = Bezier.curve(t_vals, ctrl)  # (50, 3) foot positions
```

### Generate demo GIF

```bash
pip install -e ".[demo]"
python scripts/generate_demo.py
```

## Development

```bash
pip install -e ".[dev]"
```

## License

MIT — see [LICENSE](LICENSE).
