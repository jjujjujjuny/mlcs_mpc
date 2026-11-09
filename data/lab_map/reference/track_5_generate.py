#!/usr/bin/env python3
"""
Generates track_5: a "school running track" stadium shape (straight - semicircle
- straight - semicircle) that fits inside the lab boundary with a comfortable
margin and keeps |kappa| moderate (single radius R = 1.5 m -> kappa = 0.667).

Outputs written next to this file:
  centerline.csv          (idx, x, y)
  left_boundary.csv       (idx, x, y)
  right_boundary.csv      (idx, x, y)
  curvature_profile.csv   (idx, s, x, y, kappa, abs_kappa, dkappa_ds, abs_dkappa_ds, turn_radius)
  track.json              (metadata)
  preview.png             (visual sanity check)

The output schema matches tracks/track_1..4/ so PURE_PURSUIT_TRACK.PY and
STANLEY_TRACK.PY can consume it with --track 5.
"""

import json
import os
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"

# ---- Stadium geometry (found by search over the lab boundary) -------------
CENTER_X = -0.20
CENTER_Y = -0.20
RADIUS   = 1.30          # semicircle radius [m] -> kappa = 1/R
STRAIGHT = 1.40          # straight segment length [m]
WIDTH    = 0.60          # track width (centerline to outer = 0.30)
N_POINTS = 600           # centerline samples (closed loop)
ROTATE_DEG = 20.0        # whole-track rotation, positive = CCW (tilt left)
# Orientation before rotation: straights are along the Y axis, semicircles at top/bottom.

# Where the closed loop starts (and which way it runs).
#
# Convention chosen to match tracks/track_1..4 which all start in the
# upper-LEFT region of the lab (x ~ -2, y ~ +0.5) with a curving entry
# (first samples have kappa > 0). We pick the top-LEFT of the left
# semicircle in the unrotated frame: this is the point on the left
# half-circle that sits ~135 deg from +x, so going CCW the very next
# samples are already on the upper-left curve.
START_X = CENTER_X - RADIUS * math.cos(math.radians(45))
START_Y = CENTER_Y + STRAIGHT / 2.0 + RADIUS * math.sin(math.radians(45))


def build_stadium_centerline(cx, cy, R, L, n):
    """
    Stadium with straights along ±y and semicircles centered at
    (cx, cy ± L/2). Returns (points[n,2], kappa[n], yaw[n], s[n]) with the
    loop closed implicitly (last point != first point).

    Arclength distribution:
        right straight  : 0          .. L
        top semicircle  : L          .. L + piR
        left straight   : L + piR    .. 2L + piR
        bottom semicircle: 2L + piR  .. 2L + 2piR
    """
    total = 2.0 * L + 2.0 * math.pi * R
    # Distribute samples proportional to segment length.
    s = np.linspace(0.0, total, n, endpoint=False)

    xs = np.zeros(n)
    ys = np.zeros(n)
    yaws = np.zeros(n)
    kappa = np.zeros(n)

    seg1_end = L
    seg2_end = L + math.pi * R
    seg3_end = 2 * L + math.pi * R

    for i in range(n):
        si = s[i]
        if si < seg1_end:
            # Right straight, heading +y, x = cx+R, y = cy-L/2 + si
            xs[i] = cx + R
            ys[i] = cy - L / 2.0 + si
            yaws[i] = math.pi / 2.0
            kappa[i] = 0.0
        elif si < seg2_end:
            # Top semicircle (CCW), center (cx, cy+L/2). theta=0 at +x, going up to pi at -x.
            theta = (si - seg1_end) / R
            xs[i] = cx + R * math.cos(theta)
            ys[i] = cy + L / 2.0 + R * math.sin(theta)
            yaws[i] = theta + math.pi / 2.0       # tangent direction along CCW circle
            kappa[i] = 1.0 / R                    # CCW left turn -> positive kappa
        elif si < seg3_end:
            # Left straight, heading -y, x = cx-R
            xs[i] = cx - R
            ys[i] = cy + L / 2.0 - (si - seg2_end)
            yaws[i] = -math.pi / 2.0
            kappa[i] = 0.0
        else:
            # Bottom semicircle (CCW), center (cx, cy-L/2). starts at theta=pi (-x), to 2pi (+x).
            theta = math.pi + (si - seg3_end) / R
            xs[i] = cx + R * math.cos(theta)
            ys[i] = cy - L / 2.0 + R * math.sin(theta)
            yaws[i] = theta + math.pi / 2.0
            kappa[i] = 1.0 / R

    pts = np.column_stack([xs, ys])
    yaws = np.array([math.atan2(math.sin(y), math.cos(y)) for y in yaws])
    return pts, kappa, yaws, s, total


def rotate_loop_to_start(pts, kappa, yaws, s, start_xy):
    """Rotate the index so element 0 is the point closest to start_xy."""
    sx, sy = start_xy
    d2 = (pts[:, 0] - sx) ** 2 + (pts[:, 1] - sy) ** 2
    k = int(np.argmin(d2))
    pts = np.roll(pts, -k, axis=0)
    kappa = np.roll(kappa, -k)
    yaws = np.roll(yaws, -k)
    # Re-zero arclength
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    return pts, kappa, yaws, s


def build_boundaries(pts, yaws, half_width):
    """Offset centerline ±half_width along the LEFT/RIGHT normal."""
    nx = -np.sin(yaws)
    ny = np.cos(yaws)
    left  = pts + half_width * np.column_stack([nx, ny])
    right = pts - half_width * np.column_stack([nx, ny])
    return left, right


def apply_rotation(pts, yaws, angle_rad, pivot_xy):
    """Rotate the centerline (points + yaws) by angle_rad CCW around pivot."""
    c = math.cos(angle_rad); s = math.sin(angle_rad)
    px, py = pivot_xy
    dx = pts[:, 0] - px
    dy = pts[:, 1] - py
    out = np.column_stack([px + c*dx - s*dy, py + s*dx + c*dy])
    return out, np.array([math.atan2(math.sin(y + angle_rad), math.cos(y + angle_rad)) for y in yaws])


def main():
    pts, kappa, yaws, s, total_len = build_stadium_centerline(
        CENTER_X, CENTER_Y, RADIUS, STRAIGHT, N_POINTS
    )

    # Tilt the whole track around its geometric center by ROTATE_DEG.
    pts, yaws = apply_rotation(
        pts, yaws,
        angle_rad=math.radians(ROTATE_DEG),
        pivot_xy=(CENTER_X, CENTER_Y),
    )

    # Re-rotate the index so the start point is the post-rotation projection
    # of (START_X, START_Y).
    c = math.cos(math.radians(ROTATE_DEG)); s_ = math.sin(math.radians(ROTATE_DEG))
    sx_rot = CENTER_X + c*(START_X - CENTER_X) - s_*(START_Y - CENTER_Y)
    sy_rot = CENTER_Y + s_*(START_X - CENTER_X) + c*(START_Y - CENTER_Y)
    pts, kappa, yaws, s = rotate_loop_to_start(pts, kappa, yaws, s, (sx_rot, sy_rot))

    left, right = build_boundaries(pts, yaws, WIDTH / 2.0)

    # --- Boundary containment sanity check (entire outer band must fit) -----
    with open(BOUNDARY_JSON) as f:
        bd = json.load(f)
    boundary_xy = np.array([[v["x"], v["y"]] for v in bd["boundary"]["vertices"]])
    boundary_path = Path(boundary_xy)

    all_outer = np.vstack([pts, left, right])
    in_mask = boundary_path.contains_points(all_outer)
    if not in_mask.all():
        n_out = int((~in_mask).sum())
        raise RuntimeError(
            f"{n_out} of {len(all_outer)} track-band points fall outside the "
            "lab boundary. Shrink RADIUS or STRAIGHT and re-run."
        )

    # --- Curvature profile (matches tracks/track_1..4 schema) ---------------
    abs_kappa = np.abs(kappa)
    # dkappa/ds: forward difference on the closed loop.
    s_closed = np.concatenate([s, [total_len]])
    kappa_closed = np.concatenate([kappa, [kappa[0]]])
    dkappa_ds = np.diff(kappa_closed) / np.diff(s_closed)
    abs_dkappa_ds = np.abs(dkappa_ds)
    turn_radius = np.where(abs_kappa > 1e-9, 1.0 / np.maximum(abs_kappa, 1e-12), np.inf)

    # --- Write CSVs ---------------------------------------------------------
    pd.DataFrame({
        "idx": np.arange(N_POINTS),
        "x": pts[:, 0],
        "y": pts[:, 1],
    }).to_csv(os.path.join(THIS_DIR, "centerline.csv"), index=False)

    pd.DataFrame({
        "idx": np.arange(N_POINTS),
        "x": left[:, 0],
        "y": left[:, 1],
    }).to_csv(os.path.join(THIS_DIR, "left_boundary.csv"), index=False)

    pd.DataFrame({
        "idx": np.arange(N_POINTS),
        "x": right[:, 0],
        "y": right[:, 1],
    }).to_csv(os.path.join(THIS_DIR, "right_boundary.csv"), index=False)

    pd.DataFrame({
        "idx": np.arange(N_POINTS),
        "s": s,
        "x": pts[:, 0],
        "y": pts[:, 1],
        "kappa": kappa,
        "abs_kappa": abs_kappa,
        "dkappa_ds": dkappa_ds,
        "abs_dkappa_ds": abs_dkappa_ds,
        "turn_radius": turn_radius,
    }).to_csv(os.path.join(THIS_DIR, "curvature_profile.csv"), index=False)

    # --- Write metadata -----------------------------------------------------
    meta = {
        "label": f"Stadium R={RADIUS:.2f}m L={STRAIGHT:.2f}m (school running track)",
        "shape": "stadium",
        "width": WIDTH,
        "num_points": N_POINTS,
        "path_length_m": float(total_len),
        "max_abs_kappa": float(abs_kappa.max()),
        "min_turn_radius_m": float(1.0 / max(abs_kappa.max(), 1e-9)),
        "geometry": {
            "center_x": CENTER_X,
            "center_y": CENTER_Y,
            "radius_m": RADIUS,
            "straight_m": STRAIGHT,
            "orientation": "straights_along_y_then_rotated",
            "rotate_deg_ccw": ROTATE_DEG,
            "direction": "ccw",
            "start_x": float(pts[0, 0]),
            "start_y": float(pts[0, 1]),
        },
    }
    with open(os.path.join(THIS_DIR, "track.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # --- PNG preview --------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 9))
    bx = np.append(boundary_xy[:, 0], boundary_xy[0, 0])
    by = np.append(boundary_xy[:, 1], boundary_xy[0, 1])
    ax.fill(bx, by, color="#f0f0f0", alpha=1.0, zorder=0)
    ax.plot(bx, by, color="black", lw=1.2, label="lab boundary")

    # Close the track loops for plotting.
    def closed(arr):
        return np.vstack([arr, arr[:1]])

    cl = closed(pts); lb = closed(left); rb = closed(right)
    ax.plot(lb[:, 0], lb[:, 1], color="tab:green", lw=1.5, label="left")
    ax.plot(rb[:, 0], rb[:, 1], color="tab:orange", lw=1.5, label="right")
    ax.plot(cl[:, 0], cl[:, 1], color="tab:blue", lw=2.0, label="centerline")
    ax.plot([pts[0, 0]], [pts[0, 1]], "o", color="red", markersize=10, label="start")

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(
        f"track_5 stadium  |  R={RADIUS:.2f} m  L={STRAIGHT:.2f} m  "
        f"rot={ROTATE_DEG:+.0f} deg  |  "
        f"len={total_len:.2f} m  |kappa|max={abs_kappa.max():.3f}"
    )
    ax.legend(loc="upper right")
    out_png = os.path.join(THIS_DIR, "preview.png")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)

    print(f"track_5 written. length={total_len:.3f} m, "
          f"|kappa|max={abs_kappa.max():.4f}, min R={1/abs_kappa.max():.3f} m")
    print(f"PNG preview: {out_png}")


if __name__ == "__main__":
    main()
