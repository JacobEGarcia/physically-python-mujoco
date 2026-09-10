# PHYSICALLY PYTHON (MuJoCo)

A Unitree G1 humanoid physically writes Python on a whiteboard - rebuilt with
MuJoCo and the official MuJoCo Menagerie Unitree G1 model.

Inspired by @dimentary's sim video (quoted by @chris_j_paxton: "Kind of amazed
that this works. not physically valid yet though, execution an issue. but you
could probably use RL on this as a reference trajectory").

The robot writes the Fibonacci script letter by letter:

```python
def fib(n):
  a, b = 0, 1
  for _ in range(n):
    a, b = b, a + b
  return a
```

## What's physically simulated vs scripted

Physically simulated (MuJoCo 3.x, 500 Hz):
- Full rigid-body dynamics of the 29-DoF Menagerie Unitree G1 (masses,
  inertias, meshes, joint limits, friction loss, armature from the official model)
- The Menagerie position actuators (kp=500, critically damped) that track the
  joint trajectory, including their force limits
- Gravity, the easel, the pen rigidly held in the right hand

Scripted (not physics):
- The whiteboard text path itself: letter strokes are a small hand-made vector
  font; the pen-tip Cartesian path is scripted
- The joint trajectory: computed offline by damped least-squares inverse
  kinematics over 9 joints (waist yaw/pitch + right arm), then tracked by the
  simulated actuators. Residual pen-tip error during drawing is ~1 mm
- The pelvis is welded to the world: this is an upper-body manipulation demo,
  not a balance controller (same "not physically valid yet" caveat as the post)
- Ink: stroke segments drawn on the board are revealed where the actual
  simulated pen tip passes within 12 mm of them

## Setup

```bash
pip install -r requirements.txt
./fetch_model.sh          # sparse-clones MuJoCo Menagerie (unitree_g1 only)
MUJOCO_GL=egl python3 write_python.py   # use MUJOCO_GL=osmesa or glfw as available
```

Output: `out/physically_python_mujoco.mp4` (1280x720, 30 fps, ~36 s).

Files:
- `write_python.py` - font, trajectory, offline IK, simulation, recording
- `scene/scene.xml` - world: G1 include, welded pelvis, whiteboard easel, lights
- `scene/g1_patched.xml` - generated: menagerie g1.xml + pen geoms/site
- `scene/ink_geoms.xml` - generated: one capsule per ink stroke segment
- `NOTES.md` - design notes

No physical robot was used. Both this and the original post are simulations.
