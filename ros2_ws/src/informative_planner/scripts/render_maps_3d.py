#!/usr/bin/python3.12
"""
Painel 3D (normalizado) do campo de visibilidade: ground truth + cada método.
Cada superfície é normalizada para [0,1] (divide pelo próprio máximo), então a
comparação é de FORMATO, não de magnitude; o pico original vai no título.

Uso:
  /usr/bin/python3.12 render_maps_3d.py <world>   (ex.: forest_clustered_10min)
Saída: ARTIGOS/figs/gp_maps_3d_<short>.png
"""
import csv, glob, os, sys
import numpy as np
sys.path.insert(0, "/root/mrs_ws/src/informative_planner")
from informative_planner.exploration_gp import ExplorationGP
from informative_planner import config as C
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

DEST = "/root/mrs_ws/RESULTADOS_ARTIGO"
GTDIR = "/root/mrs_ws/src/octomap_gazebo_forest/ground_truth"
R_VIS = 4.5
METHODS = [("proposed", "Proposed (RRT)"), ("myopic", "Myopic"), ("lawnmower", "Lawnmower")]


def load_trees(world):
    p = os.path.join(GTDIR, world, f"{world}_ground_truth.csv")
    return np.array([[float(r["x"]), float(r["y"])] for r in csv.DictReader(open(p))])


def soft_vis(xy, tr, rv=R_VIS, w=0.9):
    d = np.linalg.norm(xy[:, None, :] - tr[None, :, :], axis=2)
    d = np.where(d < 0.85, np.inf, d)
    return (1.0 / (1.0 + np.exp((d - rv) / w))).sum(1)


def replay_predict(pq, grid, fine):
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


def first(world, method, name):
    return sorted(glob.glob(os.path.join(DEST, world, method, "rep*", name)))


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "forest_clustered_10min"
    short = world.replace("forest_", "").replace("_10min", "")
    trees = load_trees(world)
    grid_csv = first(world, "proposed", "gaussian_feeder/gp_grid.csv")
    grid = np.array([[float(r["x"]), float(r["y"])]
                     for r in csv.DictReader(open(grid_csv[0]))]) if grid_csv else None

    b = 6.0
    x0, x1 = trees[:, 0].min() - b, trees[:, 0].max() + b
    y0, y1 = trees[:, 1].min() - b, trees[:, 1].max() + b
    xs = np.arange(x0, x1, 0.5); ys = np.arange(y0, y1, 0.5)
    X, Y = np.meshgrid(xs, ys); fine = np.column_stack([X.ravel(), Y.ravel()])

    panels = [("Ground truth $v^{*}$", soft_vis(fine, trees).reshape(X.shape))]
    for m, lbl in METHODS:
        pq = first(world, m, "point_quality.csv")
        if pq and grid is not None:
            Z = replay_predict(pq[0], grid, fine).reshape(X.shape)
            panels.append((lbl, Z))
        else:
            panels.append((lbl + " (pendente)", None))

    fig = plt.figure(figsize=(13, 11))
    for k, (ttl, Z) in enumerate(panels):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        if Z is None:
            ax.text2D(0.5, 0.5, "sem runs", ha="center", transform=ax.transAxes)
            ax.set_title(ttl, fontsize=13); continue
        pk = float(np.nanmax(Z))
        Zn = Z / pk if pk > 0 else Z          # NORMALIZADO para [0,1]
        ax.plot_surface(X, Y, Zn, cmap=cm.viridis, vmin=0, vmax=1,
                        linewidth=0, antialiased=True, rcount=80, ccount=80)
        ax.contourf(X, Y, Zn, zdir="z", offset=0, cmap=cm.viridis,
                    vmin=0, vmax=1, levels=25, alpha=0.6)
        ax.set_zlim(0, 1.05); ax.view_init(elev=34, azim=-60)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("normalized")
        ax.set_title(f"{ttl}   (peak $={pk:.1f}$ trunks)", fontsize=13, pad=0)
    fig.suptitle(f"{world}: normalized visibility field — ground truth vs. methods",
                 fontsize=16)
    fig.tight_layout()
    out = f"/root/mrs_ws/ARTIGOS/figs/gp_maps_3d_{short}.png"
    fig.savefig(out, dpi=145, bbox_inches="tight")
    print("salvo:", out)


if __name__ == "__main__":
    main()
