#!/usr/bin/env python3
"""Demo GIF: a MuJoCo UR5e moves through a smooth loop; the joint-state stream the "stack" receives is shaped by the real
armchaos fault models (freeze, noise, spoof, spike, lag) and judged by the real armdoc checks, whose verdict is drawn on
every frame. The arm you see is the arm as the stack sees it.

No ROS and no simulator process: simulated time, deterministic. Needs mujoco, pillow, numpy, and armdoc on PYTHONPATH.

  PYTHONPATH=.:../armdoc python3 examples/make_demo_gif.py [--out docs/demo_ur5e_faults.gif]
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from armchaos.faults import Sample
from armchaos.scenario import Engine, Step, make_fault
from armdoc import core

MODEL = os.path.join(os.environ.get("MUJOCO_MENAGERIE_PATH", os.path.expanduser("~/mujoco_menagerie")),
                     "universal_robots_ur5e", "scene.xml")
NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
Q0 = np.array([0.0, -1.45, 1.6, -1.7, -1.57, 0.0])
AMP = np.array([0.9, 0.35, 0.45, 0.5, 0.5, 0.9])
PERIOD = 8.0
HZ = 50.0                                 # joint_states rate
SHOW_EVERY = 8                            # one frame per 8 samples -> 6.25 fps
W, H = 400, 290
SCHEDULE = [                              # (at, for, fault, params)
    (3, 4, "freeze", {}),
    (10, 4, "noise", {"sigma": 0.02}),
    (17, 4, "spoof", {"joint": "shoulder_pan_joint", "offset": 7.0}),
    (24, 3, "spike", {"joint": "elbow_joint", "velocity": 10.0}),
    (30, 6, "lag", {"delay": 2.5}),
]
END = 38.0
# limits armdoc compares against: position from the MJCF, velocity and effort from the UR5e datasheet class (rad/s, N*m)
VEL_LIMIT, EFFORT_LIMIT = math.pi, 150.0

GREEN, AMBER, RED, GREY = (46, 125, 50), (230, 145, 20), (198, 40, 40), (70, 70, 70)


def font(size):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def true_state(t):
    ph = 2 * math.pi * t / PERIOD
    q = Q0 + AMP * np.sin(ph + np.arange(6) * 0.5)
    v = AMP * (2 * math.pi / PERIOD) * np.cos(ph + np.arange(6) * 0.5)
    return q, v


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/demo_ur5e_faults.gif")
    a = ap.parse_args(argv)
    if not os.path.exists(MODEL):
        sys.exit(f"UR5e model not found: {MODEL} (set MUJOCO_MENAGERIE_PATH)")
    model = mujoco.MjModel.from_xml_path(MODEL)
    model.vis.global_.offwidth, model.vis.global_.offheight = max(W, model.vis.global_.offwidth), max(H, model.vis.global_.offheight)
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    data = mujoco.MjData(model)
    rend = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, 0.0, 0.35]
    cam.distance, cam.azimuth, cam.elevation = 2.1, 140, -18

    engine = Engine(sorted((Step(at, dur, make_fault(name, **kw)) for at, dur, name, kw in SCHEDULE), key=lambda s: s.at))
    state = core.State(t0=0.0)
    lim = {n: dict(lower=float(model.jnt_range[i, 0]), upper=float(model.jnt_range[i, 1]), velocity=VEL_LIMIT,
                   effort=EFFORT_LIMIT, continuous=False) for i, n in enumerate(NAMES)}
    state.limits = lim
    cfg = core.Config(grace=0.0)

    shown = Q0.copy()                              # the arm as the stack sees it: the last sample that arrived
    arrived = []                                   # samples that came out of the fault chain this step
    verdict, label_of = [], {}
    frames, n = [], 0
    dt = 1.0 / HZ
    t = 0.0
    while t < END:
        started, stopped = engine.update(t)
        q, v = true_state(t)
        s = Sample(list(NAMES), list(map(float, q)), list(map(float, v)), [0.0] * 6, t)    # stamp = origin time
        out = engine.process(t, s) + engine.tick(t)
        for o in out:
            lag = t - o.stamp
            core.feed_joints(state, t, o.names, o.pos, o.vel, o.eff, lag=lag)
            if all(math.isfinite(x) for x in o.pos):
                shown = np.array(o.pos)
        if n % SHOW_EVERY == 0:
            findings = core.evaluate(state, t, cfg)
            active = [st for st in engine.steps if st.active]
            data.qpos[:6] = shown
            mujoco.mj_forward(model, data)
            rend.update_scene(data, cam)
            img = Image.fromarray(rend.render())
            frames.append(overlay(img, t, active, findings))
        t += dt
        n += 1
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=48) for f in frames]
    pal[0].save(a.out, save_all=True, append_images=pal[1:], duration=int(1000 * SHOW_EVERY / HZ), loop=0, optimize=True)
    print(f"wrote {a.out}: {len(frames)} frames, {os.path.getsize(a.out) / 1e6:.2f} MB")


def overlay(img, t, active, findings):
    d = ImageDraw.Draw(img, "RGBA")
    f1, f2, f3 = font(14), font(13), font(12)
    # top: what armchaos is doing
    d.rectangle([0, 0, W, 26], fill=(20, 20, 20, 215))
    if active:
        st = active[0]
        p = ", ".join(f"{k}={v}" for k, v in st.fault.p.items() if v not in (None, ""))
        d.text((8, 6), f"armchaos: {st.fault.name}({p})"[:60], font=f3, fill=(255, 205, 90))
    else:
        d.text((8, 5), "armchaos: no fault (healthy stream)", font=f1, fill=(170, 220, 170))
    # bottom: what armdoc says
    errs = [x for x in findings if x.level == core.ERROR]
    warns = [x for x in findings if x.level == core.WARN]
    colour = RED if errs else AMBER if warns else GREEN
    d.rectangle([0, H - 44, W, H], fill=colour + (225,))
    if findings:
        codes = ", ".join(x.code for x in (errs + warns))
        d.text((8, H - 40), "armdoc: " + codes[:46], font=f1, fill=(255, 255, 255))
        d.text((8, H - 20), (errs + warns)[0].msg[:56], font=f3, fill=(255, 240, 240))
    else:
        d.text((8, H - 40), "armdoc: OK  arm looks healthy", font=f1, fill=(255, 255, 255))
    # timeline with the fault windows
    x0, x1, y = 8, W - 8, 32
    d.rectangle([x0, y, x1, y + 4], fill=(255, 255, 255, 90))
    for at, dur, *_ in SCHEDULE:
        d.rectangle([x0 + (x1 - x0) * at / END, y, x0 + (x1 - x0) * (at + dur) / END, y + 4], fill=(255, 205, 90, 230))
    xt = x0 + (x1 - x0) * t / END
    d.rectangle([xt - 1, y - 3, xt + 1, y + 7], fill=(255, 255, 255, 255))
    d.text((W - 80, 40), f"t = {t:4.1f} s", font=f3, fill=(235, 235, 235))
    return img.convert("RGB")


if __name__ == "__main__":
    main()
