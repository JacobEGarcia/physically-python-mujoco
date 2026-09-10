# NOTES - PHYSICALLY PYTHON (MuJoCo)

## Source stack
Jacob's read on the original post: MuJoCo + the MuJoCo Menagerie Unitree model.
The author (@dimentary) has not published the code publicly as far as I could
verify (his public GitHub has no such repo), so I cannot confirm the exact
model variant he used. This build uses `unitree_g1` (29-DoF) from the current
MuJoCo Menagerie - the robot in the source video is visually a Unitree G1.

## Pipeline
1. A tiny hand-made vector stroke font covers the characters of the Fibonacci
   script. Strokes become a timed Cartesian pen-tip path on the board plane
   (draw at 0.07 m/s, transit at 0.16 m/s, press/lift pauses).
2. Damped least-squares IK (9 joints: waist yaw/pitch + 7 right-arm joints)
   solves the joint path offline at 25 Hz sampling, warm-started per sample
   from the previous solution and initialized from a raised writing pose -
   initializing from the hanging-arm keyframe falls into a bad local minimum.
   Draw-phase residual: ~1 mm.
3. The Menagerie position actuators track that joint path under full physics
   (gravity, friction loss, armature, torque limits). On top of the open-loop
   playback, an online closed-loop layer corrects the measured pen-tip error
   at every physics step: a proportional DLS-IK correction plus a slow
   integral term with anti-windup (per-step rate limit 0.004 rad, clamp
   +/-0.35 rad - a naive fast integrator wound up during the initial hold and
   flung the arm to the joint limits). Open-loop playback alone droops/lags
   ~90-125 mm; the proportional layer alone leaves ~34 mm of steady-state
   gravity droop; with the integral term the recorded draw-phase error is
   mean 8.9 mm / p95 25 mm / max 90 mm (the max is a stroke-start transient).
4. Ink: each drawn stroke segment is a thin capsule geom revealed (alpha 0->1)
   when the *actual simulated pen tip* passes within 22 mm of it; only the
   single nearest unrevealed segment registers per check, so a wobbling pen
   cannot ghost neighboring letters. The reveal event is driven by the real
   pen position; the 22 mm radius (set from the measured p95 error) means the
   board text is the physics' own handwriting - slightly wobbly, genuinely
   where the pen went.

## Deliberate simplifications (matching the post's own caveat)
- Pelvis welded to the world: no balance controller. The original post says
  "not physically valid yet though, execution an issue" - same here, this is
  the reference-trajectory view, not a validated controller.
- Pen is rigidly attached to the hand (no grip physics, no ink contact model).
- Writing path is scripted; nothing here is RL. That is literally the next
  step Paxton proposed; see the sibling project GAIT REFERENCE LAB.
