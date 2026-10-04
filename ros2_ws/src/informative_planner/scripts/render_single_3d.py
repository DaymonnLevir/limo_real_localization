#!/usr/bin/python3.12
"""
Triptico 3D para o banner (3 imagens separadas):
  1) map3d_proposed_<w>.png    -> GP aprendido (proposed), superfície NORMALIZADA [0,1]
  2) map3d_groundtruth_<w>.png -> ground truth v*, superfície NORMALIZADA [0,1]
  3) map3d_trees_<w>.png       -> floresta 3D bonita (modelo de árvore do
        create_tree_mapper_analysis: chão verde + tronco-cilindro + copa por
        espécie), SEM trajetória/detecções.

Uso: /usr/bin/python3.12 render_single_3d.py <world>   (default clustered)
"""
import csv, glob, os, sys
import numpy as np
sys.path.insert(0, "/root/mrs_ws/src/informative_planner")
from informative_planner.exploration_gp import ExplorationGP
from informative_planner import config as C
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

DEST = "/root/mrs_ws/RESULTADOS_ARTIGO"
GTDIR = "/root/mrs_ws/src/octomap_gazebo_forest/ground_truth"
FIGS = "/root/mrs_ws/ARTIGOS/figs"
R_VIS = 4.5

# paleta por espécie (tronco | copa base | copa highlight) — igual ao analyze_batch
MODEL_PALETTE = {
    "benchmark_fg_tree_2":       ("#5C3D1E", "#2D6A4F", "#52B788"),
    "benchmark_world_jean_tree": ("#6B4423", "#40916C", "#74C69D"),
    "benchmark_new_tree_1":      ("#4A2C12", "#1B4332", "#2D6A4F"),
    "benchmark_fg_tree_9":       ("#7A4F2E", "#52B788", "#95D5B2"),
}
DEFAULT_PAL = ("#5C3D1E", "#2D6A4F", "#52B788")
GRID = "#D9E0E6"


def load_forest(world):
    p = os.path.join(GTDIR, world, f"{world}_ground_truth.csv")
    with open(p, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    trees = [{
        "x": float(r["x"]), "y": float(r["y"]),
        "height_m": float(r["height_m"]),
        "crown_radius_m": float(r["crown_radius_m"]),
        "dap_m": float(r["dap_m"]),
        "model": r.get("adapter_model", ""),
    } for r in rows]
    xy = np.array([[t["x"], t["y"]] for t in trees])
    return trees, xy


def soft_vis(xy, tr, rv=R_VIS, w=0.9):
    d = np.linalg.norm(xy[:, None, :] - tr[None, :, :], axis=2)
    d = np.where(d < 0.85, np.inf, d)
    return (1.0 / (1.0 + np.exp((d - rv) / w))).sum(1)


def learned(pq_csv, grid_csv, fine):
    grid = np.array([[float(r["x"]), float(r["y"])] for r in csv.DictReader(open(grid_csv))])
    pq = pq_csv
    gp = ExplorationGP(candidates_xy=grid, length_scale=C.GP_ELL, signal_std=C.GP_SIGMA_S,
                       noise_std=C.GP_SIGMA_N, beta=C.GP_BETA,
                       observation_model=C.GP_OBSERVATION_MODEL,
                       observation_footprint_radius=C.GP_OBSERVATION_FOOTPRINT_RADIUS,
                       observation_footprint_sigma=C.GP_OBSERVATION_FOOTPRINT_SIGMA)
    last = None
    for r in csv.DictReader(open(pq)):
        p = np.array([float(r["x"]), float(r["y"])])
        if last is not None and np.allclose(p, last):
            continue
        last = p
        gp.add_observation(p, float(r["point_score"]))
    mu, _ = gp.predict(fine)
    return mu


def surf(X, Y, Z, ext, title, out):
    Zn = Z / max(float(np.nanmax(Z)), 1e-9)          # NORMALIZA para [0,1]
    fig = plt.figure(figsize=(8, 7)); ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(X, Y, Zn, cmap=cm.viridis, vmin=0, vmax=1, linewidth=0,
                    antialiased=True, rcount=110, ccount=110)
    ax.set_xlim(*ext[:2]); ax.set_ylim(*ext[2:]); ax.set_zlim(0, 1.05)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("normalized visibility")
    ax.view_init(elev=30, azim=-60)
    fig.tight_layout(); fig.savefig(out, dpi=155, bbox_inches="tight"); plt.close(fig)
    print("salvo:", out)


def _trunk_cylinder(ax, cx, cy, r, z_top, color):
    n = 10
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    xs = cx + r * np.cos(th); ys = cy + r * np.sin(th)
    verts = [[[xs[i], ys[i], 0], [xs[(i + 1) % n], ys[(i + 1) % n], 0],
              [xs[(i + 1) % n], ys[(i + 1) % n], z_top], [xs[i], ys[i], z_top]]
             for i in range(n)]
    ax.add_collection3d(Poly3DCollection(verts, facecolor=color, alpha=1.0,
                                         edgecolor="none", zorder=6))


def forest_3d(trees, ext, X, Y, MU, title, out):
    fig = plt.figure(figsize=(11, 9)); fig.patch.set_facecolor("white")
    ax = fig.add_subplot(111, projection="3d"); ax.set_facecolor("white")
    try: ax.computed_zorder = False   # respeita o zorder manual: árvores sempre à frente
    except Exception: pass
    # mapa PREDITO 2D pelo proposto, LITERALMENTE no chão (z=0), suave (α baixo)
    cs = ax.contourf(X, Y, MU, zdir="z", offset=0, cmap=cm.viridis,
                     levels=25, alpha=0.55, zorder=0)
    # árvores: cor e opacidade UNIFORMES, sempre por cima (depthshade off)
    TRUNK, CROWN = "#5C3D1E", "#2D6A4F"
    for t in trees:
        x, y, h, rc = t["x"], t["y"], t["height_m"], t["crown_radius_m"]
        _trunk_cylinder(ax, x, y, t["dap_m"] / 2.0, h * 0.55, TRUNK)
        ax.scatter([x], [y], [h * 0.72], s=380 * rc, color=CROWN, alpha=1.0,
                   edgecolors="none", depthshade=False, zorder=12)
    # eixos casados com a grade do mapa -> o plano preenche o chão exatamente
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
    ax.set_zlim(0, 9.5)
    ax.set_xlabel("x [m]", labelpad=10); ax.set_ylabel("y [m]", labelpad=10)
    ax.set_zlabel("z [m]", labelpad=7)
    try: ax.set_box_aspect([1.0, 1.0, 0.30])
    except AttributeError: pass
    ax.view_init(elev=28, azim=-52)
    for pane, col in [(ax.xaxis.pane, "#F7F7F5"), (ax.yaxis.pane, "#F7F7F5"),
                      (ax.zaxis.pane, "#F2F4F0")]:
        pane.fill = True; pane.set_facecolor(col); pane.set_edgecolor(GRID)
    ax.grid(True, linestyle=":", linewidth=0.4, alpha=0.5)
    # colorbar do mapa predito + legenda simples
    cb = fig.colorbar(cs, ax=ax, shrink=0.5, aspect=18, pad=0.02)
    cb.set_label(r"predicted visibility $\mu(x,y)$")
    ax.legend(handles=[
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#40916C",
               markeredgecolor="none", markersize=13, label="ground-truth tree"),
    ], loc="upper left", fontsize=11, framealpha=0.9)
    fig.tight_layout(); fig.savefig(out, dpi=155, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("salvo:", out)


def trajectory_2d(traj_csv, trees, ext, out):
    rows = list(csv.DictReader(open(traj_csv)))
    x = np.array([float(r["x"]) for r in rows]); y = np.array([float(r["y"]) for r in rows])
    t = np.array([float(r["time_sec"]) for r in rows])
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.scatter(trees[:, 0], trees[:, 1], s=70, facecolors="none", edgecolors="#2D6A4F",
               linewidths=1.4, zorder=2, label="ground-truth trees")
    sc = ax.scatter(x, y, c=t, cmap="viridis", s=6, zorder=3)
    ax.plot(x[0], y[0], marker="s", ms=12, color="#17324D", zorder=4, label="start")
    fig.colorbar(sc, ax=ax, label="mission time [s]")
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=9); ax.grid(True, alpha=0.25)
    fig.tight_layout(); fig.savefig(out, dpi=150, bbox_inches="tight"); plt.close(fig)
    print("salvo:", out)


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "forest_clustered_10min"
    short = world.replace("forest_", "").replace("_10min", "")
    trees, xy = load_forest(world)
    run_dir = sorted(glob.glob(os.path.join(DEST, world, "proposed", "rep*")))[0]
    print("RUN usado (proposed):", run_dir)
    pq = os.path.join(run_dir, "point_quality.csv")
    grid_csv = os.path.join(run_dir, "gaussian_feeder", "gp_grid.csv")
    traj = os.path.join(run_dir, "trajectory.csv")
    b = 6.0
    x0, x1 = xy[:, 0].min() - b, xy[:, 0].max() + b
    y0, y1 = xy[:, 1].min() - b, xy[:, 1].max() + b
    ext = (x0, x1, y0, y1)
    xs = np.arange(x0, x1, 0.5); ys = np.arange(y0, y1, 0.5)
    X, Y = np.meshgrid(xs, ys); fine = np.column_stack([X.ravel(), Y.ravel()])

    MU = learned(pq, grid_csv, fine).reshape(X.shape)
    VS = soft_vis(fine, xy).reshape(X.shape)

    surf(X, Y, MU, ext, f"Proposed — learned GP (normalized) ({short})",
         os.path.join(FIGS, f"map3d_proposed_{short}.png"))
    surf(X, Y, VS, ext, f"Ground truth $v^{{*}}$ (normalized) ({short})",
         os.path.join(FIGS, f"map3d_groundtruth_{short}.png"))
    forest_3d(trees, ext, X, Y, MU, "Predicted visibility map + ground-truth forest",
              os.path.join(FIGS, f"map3d_trees_{short}.png"))
    trajectory_2d(traj, xy, ext, os.path.join(FIGS, f"trajectory2d_proposed_{short}.png"))


if __name__ == "__main__":
    main()
