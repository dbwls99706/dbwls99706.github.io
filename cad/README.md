# Quadruped model

The robot in the hero scene is an original design, modelled parametrically with [CadQuery](https://github.com/CadQuery/cadquery).

| File | Purpose |
| --- | --- |
| `quadruped.py` | Parametric CAD model. Each rigid link (trunk, LiDAR head, hip, thigh, calf) is built in its own joint frame. |
| `build.mjs` | Simplifies the tessellated links and packs them into `../assets/quadruped.bin` (positions, normals, vertex colours, indices). |

## Rebuild

```bash
pip install cadquery
```

```bash
python quadruped.py build --step
```

```bash
npm install --no-save meshoptimizer
```

```bash
node build.mjs build ../assets/quadruped.bin
```

`--step` also writes `build/quadruped.step`, the whole robot in its standing pose, for a CAD viewer.

If you change a kinematic parameter (joint offsets, link length, standing height), update `KIN` in `../js/robot.js` to match:
the scene uses those values for inverse kinematics and for the physics body.
