#!/usr/bin/env python3
"""
PHYSICALLY PYTHON (MuJoCo) - a Unitree G1 physically writes Python on a whiteboard.

Simulation: MuJoCo 3.x + the MuJoCo Menagerie Unitree G1 (29-DoF) model.
What is physically simulated: full-body rigid-body dynamics of the G1, position
actuators (PD), damped least-squares IK tracking of the pen tip, gravity, joint
limits/friction from the Menagerie model.
What is scripted: the pen-tip Cartesian path (letter strokes), the weld that
holds the pelvis fixed (no balance controller), and the ink reveal.
"""
import os, sys, math, json, subprocess, time
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
MENAGERIE = os.environ.get('MENAGERIE_DIR', os.path.join(ROOT, 'mujoco_menagerie'))
G1_XML = os.path.join(MENAGERIE, 'unitree_g1', 'g1.xml')
PATCHED = os.path.join(ROOT, 'scene', 'g1_patched.xml')

PEN_SNIPPET = '''<geom name="pen" class="visual" type="capsule" fromto="0.02 0 0.02 0.15 0 0.02" size="0.0055" rgba="0.08 0.08 0.1 1"/>
                          <geom name="pentip_geom" class="visual" type="sphere" pos="0.15 0 0.02" size="0.006" rgba="0.9 0.15 0.15 1"/>
                          <site name="pentip" pos="0.155 0 0.02" size="0.003" rgba="0 1 0 0"/>'''

def ensure_patched():
    src = open(G1_XML).read()
    if 'name="pentip"' in src:
        open(PATCHED, 'w').write(src); return
    anchor = '<geom pos="0.0415 -0.003 0" quat="1 0 0 0" class="visual" mesh="right_rubber_hand"/>'
    assert anchor in src, "anchor not found in g1.xml"
    open(PATCHED, 'w').write(src.replace(anchor, anchor + "\n                          " + PEN_SNIPPET))

# ---------------- stroke font (unit grid, y up, baseline 0) ----------------
def arc(cx, cy, rx, ry, a0, a1, n=12):
    return [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, n)]
FONT = {
  'a': [arc(2.3,2.3,2.1,2.1,0,360), [(4.4,4.4),(4.4,0)]],
  'b': [[(0.6,7),(0.6,0)], arc(2.7,2.2,2.0,2.1,0,360)],
  'd': [arc(2.3,2.2,2.0,2.1,0,360), [(4.4,7),(4.4,0)]],
  'e': [arc(2.5,2.7,2.2,2.2,-40,265), [(0.5,2.7),(4.5,2.7)]],
  'f': [[(4.3,6.4),(3.6,7.0),(2.8,6.6),(2.5,5.6),(2.5,0)], [(1.0,4.3),(3.9,4.3)]],
  'g': [arc(2.4,3.0,2.1,2.0,0,360), [(4.5,5.0),(4.5,-1.2),(3.7,-2.2),(2.4,-2.0)]],
  'i': [[(2.5,4.4),(2.5,0)], arc(2.5,6.0,0.28,0.28,0,360,6)],
  'n': [[(0.7,4.4),(0.7,0)], [(0.7,3.2),(1.2,4.1),(2.3,4.5),(3.4,4.1),(4.0,3.0),(4.0,0)]],
  'o': [arc(2.5,2.4,2.2,2.2,0,360)],
  'r': [[(0.7,4.4),(0.7,0)], [(0.7,3.3),(1.4,4.3),(2.6,4.5),(3.6,3.9)]],
  't': [[(2.3,6.4),(2.3,1.0),(3.0,0.1),(4.0,0.2)], [(1.0,4.4),(3.6,4.4)]],
  'u': [[(0.7,4.4),(0.7,1.2),(1.6,0.1),(3.0,0.1),(4.0,1.2),(4.0,4.4)], [(4.0,4.4),(4.0,0)]],
  '0': [arc(2.5,3.5,2.2,3.0,0,360,14)],
  '1': [[(1.3,5.5),(2.7,6.8),(2.7,0)]],
  '(': [arc(4.6,3.25,3.0,3.6,110,250,10)],
  ')': [arc(0.4,3.25,3.0,3.6,-70,70,10)],
  ':': [arc(2.3,3.4,0.28,0.28,0,360,6), arc(2.3,1.3,0.28,0.28,0,360,6)],
  ',': [[(2.2,1.5),(1.7,-0.7)]],
  '=': [[(0.8,3.6),(4.2,3.6)], [(0.8,2.0),(4.2,2.0)]],
  '_': [[(0.5,0.4),(4.5,0.4)]],
  '+': [[(2.5,4.2),(2.5,1.2)], [(1.0,2.7),(4.0,2.7)]],
  ' ': [],
}

SCRIPT_LINES = [
    "def fib(n):",
    "  a, b = 0, 1",
    "  for _ in range(n):",
    "    a, b = b, a + b",
    "  return a",
]

# board geometry (world)
X_WRITE = 0.436       # board face
X_HOVER = 0.416
UNIT = 0.004          # font unit -> meters
ADV = 6 * UNIT        # char advance
LPITCH = 10.5 * UNIT  # line pitch
Y0, Z0 = 0.228, 1.372 # first char baseline origin (top line); y runs +y->-y (left->right for the camera)

V_DRAW, V_MOVE = 0.07, 0.16   # m/s
T_PRESS = 0.07

def build_trajectory():
    """Return list of segments: (t0, t1, p0(3), p1(3), drawing)."""
    segs = []
    t = 0.0
    pen = np.array([0.25, -0.15, 1.05])  # nominal start above home pose
    for row, line in enumerate(SCRIPT_LINES):
        for col, ch in enumerate(line):
            gy, gz = Y0 - col * ADV, Z0 - row * LPITCH
            for stroke in FONT[ch]:
                pts = [np.array([X_WRITE, gy - u * UNIT, gz + v * UNIT]) for (u, v) in stroke]
                # hover to stroke start
                hp = pts[0].copy(); hp[0] = X_HOVER
                d = np.linalg.norm(hp - pen)
                segs.append((t, t + max(d / V_MOVE, 0.05), pen.copy(), hp, False)); t += max(d / V_MOVE, 0.05)
                pen = hp
                # press
                segs.append((t, t + T_PRESS, pen.copy(), pts[0].copy(), False)); t += T_PRESS
                pen = pts[0].copy()
                # draw
                for a, b in zip(pts[:-1], pts[1:]):
                    L = np.linalg.norm(b - a)
                    dt = max(L / V_DRAW, 0.02)
                    segs.append((t, t + dt, a.copy(), b.copy(), True)); t += dt
                    pen = b.copy()
                # lift
                lif = pen.copy(); lif[0] = X_HOVER
                segs.append((t, t + T_PRESS, pen.copy(), lif, False)); t += T_PRESS
                pen = lif
    return segs, t

def build_ink_xml(segs):
    lines = []
    i = 0
    for s0, s1, p0, p1, dr in segs:
        if not dr: continue
        lines.append(f'    <geom name="ink_{i}" type="capsule" fromto="{p0[0]-0.0015:.5f} {p0[1]:.5f} {p0[2]:.5f} '
                     f'{p1[0]-0.0015:.5f} {p1[1]:.5f} {p1[2]:.5f}" size="0.0011" '
                     f'rgba="0.04 0.05 0.09 0" contype="0" conaffinity="0"/>')
        i += 1
    open(os.path.join(ROOT, 'scene', 'ink_geoms.xml'), 'w').write("<mujoco>\n" + "\n".join(lines) + "\n</mujoco>\n")

def main():
    ensure_patched()
    segs, T_END = build_trajectory()
    build_ink_xml(segs)
    import mujoco

    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, 'scene', 'scene.xml'))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.qpos[2] = 0.793  # ensure pelvis height matches weld
    mujoco.mj_forward(m, d)
    print(f"trajectory: {len(segs)} segments, {T_END:.1f}s", flush=True)

    ink_name = []
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        if n and n.startswith('ink_'):
            ink_name.append((int(n.split('_')[1]), i))
    ink_name.sort()
    ink_ids = [gid for _, gid in ink_name]
    draw_segs = [s for s in segs if s[4]]
    assert len(ink_ids) == len(draw_segs), f"ink geoms {len(ink_ids)} != drawn segments {len(draw_segs)}"
    ink_mid = np.array([0.5 * (s[2] + s[3]) for s in draw_segs])

    arm_joints = ['waist_yaw_joint', 'waist_pitch_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                  'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
                  'right_wrist_pitch_joint', 'right_wrist_yaw_joint']
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in arm_joints]
    dof = [m.jnt_dofadr[j] for j in jid]
    qadr = [m.jnt_qposadr[j] for j in jid]
    act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in arm_joints]
    lo = np.array([m.actuator_ctrlrange[a][0] for a in act]); hi = np.array([m.actuator_ctrlrange[a][1] for a in act])
    key_q = np.array([m.key_qpos[0][q] for q in qadr])
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'pentip')

    hold, tail = 1.5, 2.5
    total = hold + T_END + tail
    TS = 0.04  # IK sample period
    times = np.arange(0, total, TS)

    def seg_at(tt):
        lo_, hi_ = 0, len(segs) - 1
        while lo_ < hi_:
            mid = (lo_ + hi_) // 2
            if segs[mid][1] < tt: lo_ = mid + 1
            else: hi_ = mid
        return segs[lo_]

    def target_at(t_sim):
        tw = t_sim - hold
        if tw < 0: return segs[0][2]
        if tw >= T_END: return segs[-1][3]
        s0, s1, p0, p1, dr = seg_at(tw)
        a = min(max((tw - s0) / max(s1 - s0, 1e-6), 0.0), 1.0)
        a = a * a * (3 - 2 * a)
        return p0 + (p1 - p0) * a

    def solve_ik(corr, warm=None):
        """Offline DLS-IK over the 9 writing joints, against targets + corr (ILC correction)."""
        dk = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, dk, 0)
        jacp = np.zeros((3, m.nv))
        qs = np.zeros((len(times), len(arm_joints)))
        q = key_q.copy()
        q[arm_joints.index('right_shoulder_pitch_joint')] = -1.0
        q[arm_joints.index('right_elbow_joint')] = 1.2
        if warm is not None: q = warm.copy()
        worst = 0.0
        for ti, t in enumerate(times):
            tgt = target_at(t) + corr[ti]
            for _ in range(120):
                for i, qa in enumerate(qadr): dk.qpos[qa] = q[i]
                mujoco.mj_forward(m, dk)
                err = tgt - dk.site_xpos[sid]
                n = np.linalg.norm(err)
                if n > worst and t > hold: worst = n
                if n < 3e-4: break
                mujoco.mj_jacSite(m, dk, jacp, None, sid)
                J = jacp[:, dof]
                dq = J.T @ np.linalg.solve(J @ J.T + 0.01 ** 2 * np.eye(3), err)
                dq += 0.001 * (key_q - q)
                q = np.clip(q + 0.5 * dq, lo, hi)
            qs[ti] = q
            if ti % 150 == 0: print(f"  IK {ti}/{len(times)}", flush=True)
        print(f"  IK solve done, worst residual after approach: {worst*1000:.1f} mm", flush=True)
        return qs

    W, H, FPS = 1280, 720, 30
    dt = m.opt.timestep

    def run_sim(qs, corr=None, render_path=None):
        """Play qs back through the simulated position actuators with an online
        closed-loop DLS-IK correction layer. Records pen-tip error at the TS grid
        for ILC. Optionally renders an mp4."""
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[2] = 0.793
        for i in range(m.nu):
            j = m.actuator_trnid[i, 0]
            d.ctrl[i] = m.key_qpos[0][m.jnt_qposadr[j]]
        mujoco.mj_forward(m, d)
        for gid in ink_ids: m.geom_rgba[gid][3] = 0.0
        ink_done = np.zeros(len(ink_ids), dtype=bool)
        err_rec = np.zeros((len(times), 3))
        draw_errs = []
        jacp = np.zeros((3, m.nv))
        qoff = np.zeros(len(arm_joints))  # slow integral action with anti-windup
        renderer = None; ff = None; cam = None
        if render_path:
            renderer = mujoco.Renderer(m, height=H, width=W)
            cam = mujoco.MjvCamera()
            cam.azimuth, cam.elevation, cam.distance = 40, -16, 1.12
            cam.lookat[:] = [0.30, -0.03, 1.12]
            ff = subprocess.Popen(['ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                                   '-r', str(FPS), '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                   '-crf', '20', '-preset', 'veryfast', render_path], stdin=subprocess.PIPE)
        steps_per_frame = int(round((1.0 / FPS) / dt))
        nsteps = int(total / dt)
        next_sample = 0
        t0wall = time.time()
        for k in range(nsteps):
            t_sim = k * dt
            tq = t_sim
            qi = min(int(tq / TS), len(times) - 1)
            fr = (tq / TS) - qi
            if qi < len(times) - 1:
                qd = qs[qi] * (1 - fr) + qs[qi + 1] * fr
            else:
                qd = qs[qi]
            # online closed-loop correction: DLS IK on measured pen-tip error
            # (against the ILC-corrected target so it does not fight the playback)
            tgt_now = target_at(t_sim)
            if corr is not None:
                tgt_now = tgt_now + (corr[qi] * (1 - fr) + corr[min(qi + 1, len(times) - 1)] * fr)
            err_now = tgt_now - d.site_xpos[sid]
            mujoco.mj_jacSite(m, d, jacp, None, sid)
            J = jacp[:, dof]
            dq = J.T @ np.linalg.solve(J @ J.T + 0.02 ** 2 * np.eye(3), err_now)
            qd = np.clip(qd + np.clip(dq, -0.12, 0.12), lo, hi)  # proportional layer
            qoff += np.clip(0.15 * dq, -0.004, 0.004)
            np.clip(qoff, -0.35, 0.35, out=qoff)  # anti-windup
            qd = np.clip(qd + qoff, lo, hi)
            for i, a in enumerate(act): d.ctrl[a] = qd[i]
            mujoco.mj_step(m, d)
            while next_sample < len(times) and t_sim + dt > times[next_sample]:
                err_rec[next_sample] = target_at(times[next_sample]) - d.site_xpos[sid]
                next_sample += 1
            tw = t_sim - hold
            if 0 <= tw <= T_END:
                s0, s1, p0, p1, dr = seg_at(min(tw, segs[-1][1]))
                if dr:
                    a2 = min(max((tw - s0) / max(s1 - s0, 1e-6), 0.0), 1.0)
                    tgt = p0 + (p1 - p0) * a2
                    draw_errs.append(np.linalg.norm(tgt - d.site_xpos[sid]))
                    if k % 5 == 0 and not ink_done.all():
                        d2 = ((ink_mid - d.site_xpos[sid]) ** 2).sum(axis=1)
                        jj = int(np.argmin(d2))  # nearest segment only (anti-ghosting)
                        if d2[jj] < globals().get('REVEAL_R', 0.016) ** 2 and not ink_done[jj]:
                            m.geom_rgba[ink_ids[jj]][3] = 1.0
                            ink_done[jj] = True
            if ff is not None and k % steps_per_frame == 0:
                renderer.update_scene(d, camera=cam)
                ff.stdin.write(renderer.render().tobytes())
            if k % 5000 == 0:
                print(f"  t={t_sim:.1f}/{total:.1f} wall={time.time()-t0wall:.0f}s", flush=True)
        if ff is not None:
            ff.stdin.close(); ff.wait()
        de = np.array(draw_errs) if draw_errs else np.zeros(1)
        stats = dict(mean=float(de.mean()), p95=float(np.percentile(de, 95)), max=float(de.max()),
                     ink=int(ink_done.sum()), nink=len(ink_ids), wall=time.time() - t0wall)
        return err_rec, stats

    # ---- iterative learning control: measure execution error, feed it back into the targets ----
    corr = np.zeros((len(times), 3))
    qs = None; warm = None
    for ilc in range(1):
        print(f"ILC pass {ilc}: solving IK...", flush=True)
        qs = solve_ik(corr, warm)
        err_rec, stats = run_sim(qs, corr)
        warm = qs[-1]
        print(f"ILC pass {ilc}: draw err mean {stats['mean']*1000:.1f} mm, p95 {stats['p95']*1000:.1f} mm, "
              f"max {stats['max']*1000:.1f} mm, ink {stats['ink']}/{stats['nink']} (wall {stats['wall']:.0f}s)", flush=True)
        if stats['mean'] < 0.008 and stats['ink'] >= int(0.97 * stats['nink']):
            break
        corr += 0.7 * err_rec

    REVEAL_R = float(np.clip(1.15 * stats['p95'], 0.016, 0.022))
    print(f"ink reveal radius: {REVEAL_R*1000:.1f} mm", flush=True)
    globals()['REVEAL_R'] = REVEAL_R
    out = os.path.join(ROOT, 'out'); os.makedirs(out, exist_ok=True)
    if os.environ.get('SKIP_RENDER'):
        print("SKIP_RENDER set, stopping after probe", flush=True)
        return
    print("rendering final video...", flush=True)
    _, stats = run_sim(qs, corr, render_path=os.path.join(out, 'physically_python_mujoco.mp4'))
    print(f"done. draw err mean {stats['mean']*1000:.1f} mm, p95 {stats['p95']*1000:.1f} mm, "
          f"max {stats['max']*1000:.1f} mm, ink revealed {stats['ink']}/{stats['nink']}", flush=True)

if __name__ == '__main__':
    main()
