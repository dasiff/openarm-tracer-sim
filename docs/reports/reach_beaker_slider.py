"""Beaker-centred reach view: for the scene's beaker, how many approach bins reach it from each floor spot, at each pedestal command.

Filtered maps only (data/reach_map_before_bench). Left arm only; the right arm is the mirror with an offset. Heading straight (90 deg):
torso +x points along world +y. Rotate the picture to read other headings.

Modes (repo root, project venv):
    python docs/reports/reach_beaker_slider.py --geometry
        One MuJoCo process. Loads the scene, reads the left shoulder body from the model at pedestal READ_PEDESTAL_M (base frame),
        checks it against stance_finder.robot_geometry (the stance selection's source), and writes data/reach_heatmap/shoulder_q016.json.
    python docs/reports/reach_beaker_slider.py --html
        No MuJoCo. Counts, per floor spot and pedestal command, the approach bins whose map holds the beaker's grasp point, and writes
        docs/reports/reach_beaker_slider.html (slider over the pedestal command).

Map indexing (src/reach_map.py): file left_bin{k}.npz, k = 0..11; bin k is centred k * 30 deg from the torso's +x axis (turning left).
A map holds `cubes[ix, iy, iz]` (bool, 1 cm cubes) in the SHOULDER frame; `origin` is the shoulder-frame corner of cube (0, 0, 0).
A point P (world) for a base at (X, Y) with heading h and pedestal command q:
    d = P_xy - shoulder_xy(X, Y, h)          shoulder_xy = base xy + R(h) @ (shoulder offset in the base frame, xy)
    rx, ry = R(-h) d                         (the rotation of stance_finder._region)
    zr = P_z - shoulder_z(q)                 shoulder_z(q) = base_z + shoulder height in the base frame at READ_PEDESTAL_M + (q - READ_PEDESTAL_M)
    ix = floor((rx - origin_x) / cube), iy, iz likewise (cube = 0.01 m); reached from bin k if cubes[ix, iy, iz].
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
os.chdir(PROJECT_ROOT)

import numpy as np  # noqa: E402

GEOMETRY_JSON = Path("data/reach_heatmap/shoulder_q016.json")
HTML = Path("docs/reports/reach_beaker_slider.html")
MAPS_DIR = "data/reach_map_before_bench"           # the filtered maps (bench in the map)
N_BINS = 12
ARM = "left"
HEADING_DEG = 90.0                                 # straight: torso +x along world +y (David)
READ_PEDESTAL_M = 0.16                             # David: read the shoulder from the model at this pedestal height
FLOOR_STEP_M = 0.05                                # floor spot grid (world)
FLOOR_X = (-3.0, 3.0)
FLOOR_Y = (-2.0, 2.0)


def geometry():
    """One MuJoCo process: the left shoulder in the base frame at READ_PEDESTAL_M, read from the model; checked against robot_geometry."""
    import mujoco

    from src import robot_specs
    from src.stance_finder import robot_geometry
    from src.task_beaker_hotplate import load_task
    from src.task_config import TASK_CFG

    sim = load_task()
    m, d = sim.model, sim.data
    rg = robot_geometry(m)                          # the stance selection's source (it leaves the pedestal collapsed)
    for jn in robot_specs.PEDESTAL_SPECS["joints"]:
        d.qpos[m.joint(jn).qposadr[0]] = READ_PEDESTAL_M
    mujoco.mj_forward(m, d)
    base_R = d.body("base_link").xmat.reshape(3, 3)
    base_p = d.body("base_link").xpos
    shoulder_base = base_R.T @ (d.body(f"openarm_{ARM}_link0").xpos - base_p)     # the shoulder in the base frame (heading 0 = the base's own frame)
    torso_base = base_R.T @ (d.body("openarm_body_link0").xpos - base_p)
    rg_offset = np.asarray(rg["shoulder"][ARM], float)                            # the same offset from robot_geometry (torso frame)
    check_xy = float(np.linalg.norm((shoulder_base - torso_base)[:2] - rg_offset[:2]))
    check_z = float(abs(shoulder_base[2] - rg["torso_z0"] - READ_PEDESTAL_M - rg_offset[2]))     # heights above the base origin (no floor offset)
    beaker = d.body("task_beaker").xpos[:2].tolist()
    g = m.geom("bench_top").id
    cx, cy, cz = [float(v) for v in d.geom_xpos[g]]
    hx, hy, hz = [float(v) for v in m.geom_size[g]]
    out = {
        "read_pedestal_m": READ_PEDESTAL_M,
        "shoulder_base_frame": [float(v) for v in shoulder_base],
        "torso_base_frame": [float(v) for v in torso_base],
        "robot_geometry_shoulder_torso_frame": [float(v) for v in rg_offset],
        "robot_geometry_base_z": float(rg["base_z"]),
        "robot_geometry_torso_z0": float(rg["torso_z0"]),
        "check_xy_mm": 1000 * check_xy,
        "check_z_mm": 1000 * check_z,
        "beaker_xy": beaker,
        "beaker_xy_task_cfg": [float(v) for v in TASK_CFG["beaker_xy"]],
        "bench": {"x": [cx - hx, cx + hx], "y": [cy - hy, cy + hy], "surface_z": cz + hz},
    }
    GEOMETRY_JSON.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(GEOMETRY_JSON) + ".tmp"
    Path(tmp).write_text(json.dumps(out, indent=1))
    os.replace(tmp, GEOMETRY_JSON)
    print(json.dumps(out, indent=1), flush=True)


def html():
    from src import bench_stance as bs
    from src import robot_specs
    from src.task_config import BEAKER_CFG, PICK_PLACE_CFG, STANCE_CFG

    g = json.loads(GEOMETRY_JSON.read_text())
    maps = bs.load_maps(MAPS_DIR, sides=(ARM,))
    assert len(maps) == N_BINS, f"expected {N_BINS} {ARM} maps in {MAPS_DIR}, got {len(maps)}"
    bench = g["bench"]
    z_grasp = bench["surface_z"] + PICK_PLACE_CFG["grasp_height_frac"] * BEAKER_CFG["height"]
    rel = np.asarray(g["shoulder_base_frame"], float)
    base_z = g["robot_geometry_base_z"]
    qs = np.round(np.arange(robot_specs.PEDESTAL_SPECS["lowest_command"], robot_specs.PEDESTAL_SPECS["range"][1] + 1e-9, STANCE_CFG["pedestal_step"]), 4)
    h = math.radians(HEADING_DEG)
    c, s = math.cos(h), math.sin(h)
    xs = np.arange(FLOOR_X[0], FLOOR_X[1], FLOOR_STEP_M) + FLOOR_STEP_M / 2
    ys = np.arange(FLOOR_Y[0], FLOOR_Y[1], FLOOR_STEP_M) + FLOOR_STEP_M / 2
    X, Y = np.meshgrid(xs, ys)
    X, Y = X.ravel(), Y.ravel()
    inside_bench = (X >= bench["x"][0]) & (X <= bench["x"][1]) & (Y >= bench["y"][0]) & (Y <= bench["y"][1])
    bx, by = g["beaker_xy"]
    # the shoulder's world xy for a base at (X, Y) facing h; then the beaker relative to it, turned into the shoulder frame (stance_finder._region)
    Sx = X + c * rel[0] - s * rel[1]
    Sy = Y + s * rel[0] + c * rel[1]
    dx, dy = bx - Sx, by - Sy
    rx, ry = c * dx + s * dy, -s * dx + c * dy
    frames = []
    for q in qs:
        shoulder_z = base_z + rel[2] + (q - READ_PEDESTAL_M)
        zr = z_grasp - shoulder_z
        count = np.zeros(X.size, int)
        for k in range(N_BINS):
            rm = maps[(ARM, k)]
            iz = int(np.floor((zr - rm.origin[2]) / rm.cube))
            if not 0 <= iz < rm.cubes.shape[2]:
                continue
            jx = np.floor((rx - rm.origin[0]) / rm.cube).astype(int)
            jy = np.floor((ry - rm.origin[1]) / rm.cube).astype(int)
            inb = (jx >= 0) & (jx < rm.cubes.shape[0]) & (jy >= 0) & (jy < rm.cubes.shape[1])
            count += (rm.cubes[:, :, iz][np.clip(jx, 0, rm.cubes.shape[0] - 1), np.clip(jy, 0, rm.cubes.shape[1] - 1)] & inb).astype(int)
        count[inside_bench] = -1
        frames.append([int(v) for v in count])
        print(f"pedestal {q:.2f} m: max {count.max()} bins, reachable spots {int((count > 0).sum())}", flush=True)
    payload = {
        "title": f"Beaker at ({bx:.3f}, {by:.3f}) m, grasp point {z_grasp:.4f} m, heading {HEADING_DEG:.0f} deg, left arm, filtered maps",
        "beaker_xy": [bx, by],
        "grasp_z": z_grasp,
        "bench": bench,
        "bench_top_z": bench["surface_z"],
        "grid": {"x0": float(xs[0]), "y0": float(ys[0]), "step": FLOOR_STEP_M, "nx": len(xs), "ny": len(ys)},
        "pedestals": [float(q) for q in qs],
        "frames": frames,
        "maps": MAPS_DIR,
        "read_pedestal_m": READ_PEDESTAL_M,
        "n_bins": N_BINS,
    }
    HTML.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(HTML) + ".tmp"
    Path(tmp).write_text(TEMPLATE.replace("__DATA__", json.dumps(payload, separators=(",", ":"))), encoding="utf-8")
    os.replace(tmp, HTML)
    print("wrote", HTML, flush=True)


TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Beaker reach slider</title>
<style>
:root { --fg: #1d1d1f; --muted: #6e6e73; --bg: #ffffff; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --fg: #f5f5f7; --muted: #a1a1a6; --bg: #1c1c1e; } }
:root[data-theme="dark"] { --fg: #f5f5f7; --muted: #a1a1a6; --bg: #1c1c1e; }
body { margin: 0; padding: 16px; background: var(--bg); color: var(--fg); font: 14px/1.4 system-ui, sans-serif; }
h1 { font-size: 17px; margin: 0 0 6px; }
p { margin: 4px 0; color: var(--muted); }
#wrap { max-width: 1000px; }
canvas { width: 100%; height: auto; display: block; background: #fff; border-radius: 6px; }
#controls { display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin: 10px 0; }
input[type=range] { flex: 1 1 320px; }
#readout { font-variant-numeric: tabular-nums; min-width: 300px; }
#legend { display: flex; align-items: center; gap: 6px; font-size: 12px; color: var(--muted); }
#legend .sw { width: 22px; height: 14px; display: inline-block; }
#hover { min-height: 1.4em; font-variant-numeric: tabular-nums; }
</style></head>
<body><div id="wrap">
<h1 id="title"></h1>
<p>Each floor spot (the robot's base origin) is coloured by how many of the 12 approach bins reach the beaker's grasp point from that spot, at the pedestal command on the slider. White = spot is over the bench top (not allowed). Grey = no bin reaches. Bench outline in white; the beaker is the red cross.</p>
<div id="controls">
  <input id="slider" type="range" min="0" max="0" step="1" value="0">
  <div id="readout"></div>
</div>
<canvas id="plot"></canvas>
<div id="legend"></div>
<div id="hover"></div>
<p>Pedestal command x is in metres (the stance search's grid). The readout also gives x minus the bench top height (0.96 m), since that was asked for; it is not used in the maps.</p>
</div>
<script>
const DATA = __DATA__;
const PALETTE = ["#2b2b2b","#3b1a5c","#4b2a80","#4a3fa0","#3d5bb0","#2e77b5","#2593b0","#2aa9a6","#4cbb94","#88c97a","#c2d35a","#eedd4e","#fbf05e"];
const BG = "#d9d9d9";
const cv = document.getElementById("plot");
const ctx = cv.getContext("2d");
const G = DATA.grid;
const W = 1000, H = Math.round(W * G.ny / G.nx);
cv.width = W; cv.height = H;
const sx = W / (G.nx * G.step), sy = H / (G.ny * G.step);
const toPx = (x, y) => [(x - G.x0) * sx, H - (y - G.y0) * sy];
const slider = document.getElementById("slider");
slider.max = DATA.frames.length - 1;
document.getElementById("title").textContent = DATA.title;
const legend = document.getElementById("legend");
legend.innerHTML = "<span>0</span>" + PALETTE.map(c => `<span class="sw" style="background:${c}"></span>`).join("") + "<span>12 bins</span>";
function draw(i) {
  const counts = DATA.frames[i];
  for (let iy = 0; iy < G.ny; iy++) for (let ix = 0; ix < G.nx; ix++) {
    const v = counts[iy * G.nx + ix];
    ctx.fillStyle = v < 0 ? "#ffffff" : v === 0 ? BG : PALETTE[Math.min(v, 12)];
    const [px, py] = toPx(G.x0 + ix * G.step, G.y0 + (iy + 1) * G.step);
    ctx.fillRect(px, py, Math.ceil(G.step * sx) + 1, Math.ceil(G.step * sy) + 1);
  }
  const b = DATA.bench;
  const [x0, y0] = toPx(b.x[0], b.y[1]), [x1, y1] = toPx(b.x[1], b.y[0]);
  ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 3; ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
  const [bx, by] = toPx(DATA.beaker_xy[0], DATA.beaker_xy[1]);
  ctx.strokeStyle = "#e0203a"; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(bx - 9, by); ctx.lineTo(bx + 9, by); ctx.moveTo(bx, by - 9); ctx.lineTo(bx, by + 9); ctx.stroke();
  const q = DATA.pedestals[i];
  document.getElementById("readout").textContent = `pedestal x = ${q.toFixed(2)} m   (x - bench top ${DATA.bench_top_z.toFixed(2)} m = ${(q - DATA.bench_top_z).toFixed(2)} m)`;
}
slider.addEventListener("input", () => draw(+slider.value));
cv.addEventListener("mousemove", ev => {
  const r = cv.getBoundingClientRect();
  const px = (ev.clientX - r.left) * W / r.width, py = (ev.clientY - r.top) * H / r.height;
  const ix = Math.floor(px / sx), iy = Math.floor((H - py) / sy);
  if (ix < 0 || iy < 0 || ix >= G.nx || iy >= G.ny) { document.getElementById("hover").textContent = ""; return; }
  const v = DATA.frames[+slider.value][iy * G.nx + ix];
  const x = G.x0 + (ix + 0.5) * G.step, y = G.y0 + (iy + 0.5) * G.step;
  document.getElementById("hover").textContent = v < 0 ? `base at (${x.toFixed(2)}, ${y.toFixed(2)}) m: over the bench top` : `base at (${x.toFixed(2)}, ${y.toFixed(2)}) m: ${v} of 12 bins reach the beaker`;
});
draw(0);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--geometry", action="store_true", help="one MuJoCo process: read the shoulder at READ_PEDESTAL_M")
    ap.add_argument("--html", action="store_true", help="counts and the HTML slider (no MuJoCo)")
    a = ap.parse_args()
    if a.geometry:
        geometry()
    if a.html:
        html()
    if not (a.geometry or a.html):
        ap.error("pass --geometry or --html")


if __name__ == "__main__":
    main()
