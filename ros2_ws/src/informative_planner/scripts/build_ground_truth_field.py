#!/usr/bin/python3.12
"""
build_ground_truth_field.py
---------------------------
Reconstrói o CAMPO VERDADEIRO de visibilidade v*(x) para cada cenário de
benchmark e deixa tudo pronto para calcular RMSE e MLL do GP contra ele.

A variável de interesse do GP é o `point_score`, definido pelo tree-mapper
(tree_detector_node.py) como o NÚMERO DE TRONCOS DETECTADOS pelo RANSAC no
scan atual a partir da pose do drone. Logo, o ground truth natural é:

    v*(x) = #{ árvores do ground truth cujo tronco está a distância
              horizontal <= r_vis da pose x, e >= self-filter (0.85 m) }

O alcance FÍSICO do LiDAR (~20 m, mapplan_config.yaml) NÃO é o alcance de
DETECÇÃO de tronco: um tronco fino (~0.2 m) numa fatia de 30 cm de altura
(slice_z in [1.15, 1.45]) só rende >= dbscan_min_samples=10 pontos a curta
distância. Por isso r_vis é CALIBRADO pelos dados (observations.csv), dentro
dos limites do config: [self_filter, lidar_max_range] = [0.85, 20] m.

Uso:
  # calibra r_vis (global), gera v* + imagem por cenário, salva .npz:
  /usr/bin/python3.12 build_ground_truth_field.py build

  # avalia RMSE/MLL do GP de todos os runs (informative vs lawnmower):
  /usr/bin/python3.12 build_ground_truth_field.py eval
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------
# Caminhos e constantes vindas das configs do sistema (ver docstring)
# --------------------------------------------------------------------------
PKG = "/root/mrs_ws/src/octomap_gazebo_forest"
RESULTS = os.path.join(PKG, "results")
GT_DIR = os.path.join(PKG, "ground_truth")
OUT_DIR = os.path.join(GT_DIR, "_visibility_fields")  # imagens + .npz

SCENARIOS = [
    "forest_bands_10min",
    "forest_clustered_10min",
    "forest_diverse_10min",
    "forest_small_random_10min",
]

SELF_FILTER_M = 0.85     # tree_detector_node: self_filter_xy_radius
LIDAR_MAX_RANGE_M = 20.0  # mapplan_config.yaml: sensor_0.max_range
R_GRID = np.arange(1.0, 12.01, 0.25)  # candidatos a r_vis para a calibração


# --------------------------------------------------------------------------
# IO
# --------------------------------------------------------------------------
def load_ground_truth(scenario: str) -> np.ndarray:
    """Retorna (N, 3): x, y, dap_m das árvores do ground truth."""
    path = os.path.join(GT_DIR, scenario, f"{scenario}_ground_truth.csv")
    trees = []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            trees.append([
                float(row["x"]),
                float(row["y"]),
                float(row.get("dap_m", "nan") or "nan"),
            ])
    return np.asarray(trees, dtype=float)


def load_gp_grid(path: str):
    """Retorna (xy (M,2), mean (M,), std (M,))."""
    xy, mean, std = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            xy.append([float(row["x"]), float(row["y"])])
            mean.append(float(row["mean"]))
            std.append(float(row["std"]))
    return np.asarray(xy), np.asarray(mean), np.asarray(std)


def _runs(scenario: str, filename: str):
    """Todos os `filename` sob run dirs desse cenário (informative + lawnmower)."""
    pat = os.path.join(RESULTS, "**", f"{scenario}_*", "**", filename)
    return sorted(glob.glob(pat, recursive=True))


def pool_observations(scenario: str) -> np.ndarray:
    """Junta (x, y, point_score) de todos os observations.csv do cenário."""
    obs = []
    for f in _runs(scenario, "observations.csv"):
        with open(f) as fh:
            for row in csv.DictReader(fh):
                obs.append([
                    float(row["x"]),
                    float(row["y"]),
                    float(row["point_score"]),
                ])
    return np.asarray(obs, dtype=float)


# --------------------------------------------------------------------------
# Modelo de visibilidade
# --------------------------------------------------------------------------
def count_within(poses_xy: np.ndarray, trees_xy: np.ndarray, r: float) -> np.ndarray:
    """#{ árvores com self_filter <= dist <= r } por pose. poses (M,2)."""
    d = np.linalg.norm(poses_xy[:, None, :] - trees_xy[None, :, :], axis=2)
    within = (d <= r) & (d >= SELF_FILTER_M)
    return within.sum(axis=1).astype(float)


def fit_visibility_radius(obs: np.ndarray, trees_xy: np.ndarray):
    """Ajusta r_vis minimizando RMSE(#dentro de r, point_score observado)."""
    poses, y = obs[:, :2], obs[:, 2]
    best = None
    for r in R_GRID:
        pred = count_within(poses, trees_xy, r)
        rmse = math.sqrt(float(np.mean((pred - y) ** 2)))
        bias = float(np.mean(pred - y))
        corr = float(np.corrcoef(pred, y)[0, 1]) if np.std(pred) > 0 else 0.0
        if best is None or rmse < best["rmse"]:
            best = {"r_vis": float(r), "rmse": rmse, "bias": bias, "corr": corr}
    return best


def true_field(xy: np.ndarray, trees_xy: np.ndarray, r_vis: float) -> np.ndarray:
    """v*(x) na grade xy (contagem dura, usada nas MÉTRICAS)."""
    return count_within(xy, trees_xy, r_vis)


def soft_visibility(xy: np.ndarray, trees_xy: np.ndarray, r_vis: float,
                    width: float = 0.9) -> np.ndarray:
    """
    Versão CONTÍNUA da visibilidade, só para VISUALIZAÇÃO: cada árvore
    contribui com uma probabilidade de detecção que decai suavemente em
    torno de r_vis (logística), em vez do corte binário. O valor esperado
    ~= a contagem dura, mas o campo é liso (sem discos com borda).
    """
    d = np.linalg.norm(xy[:, None, :] - trees_xy[None, :, :], axis=2)
    d = np.where(d < SELF_FILTER_M, np.inf, d)
    p = 1.0 / (1.0 + np.exp((d - r_vis) / max(width, 1e-6)))
    return p.sum(axis=1)


# --------------------------------------------------------------------------
# Métricas (Popović 2020): RMSE e MLL do posterior GP vs. v*
# --------------------------------------------------------------------------
def rmse(mu: np.ndarray, vstar: np.ndarray) -> float:
    return math.sqrt(float(np.mean((mu - vstar) ** 2)))


def mll(mu: np.ndarray, sigma: np.ndarray, vstar: np.ndarray) -> float:
    """Mean Log Loss: -log N(v* | mu, sigma^2), média sobre as células."""
    s2 = np.maximum(sigma ** 2, 1e-6)
    return float(np.mean(0.5 * np.log(2.0 * math.pi * s2) + (vstar - mu) ** 2 / (2.0 * s2)))


# --------------------------------------------------------------------------
# Imagem
# --------------------------------------------------------------------------
def _reshape(xy: np.ndarray, vals: np.ndarray):
    xs = np.unique(xy[:, 0])
    ys = np.unique(xy[:, 1])
    idx = {(round(x, 3), round(y, 3)): v for (x, y), v in zip(xy, vals)}
    Z = np.full((len(ys), len(xs)), np.nan)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            Z[j, i] = idx.get((round(x, 3), round(y, 3)), np.nan)
    return xs, ys, Z


def save_ground_truth_image(scenario, trees, r_vis, path, bounds=None, res=0.15):
    """Renderiza a visibilidade (campo CONTÍNUO) e salva PNG com as árvores."""
    tx, ty = trees[:, 0], trees[:, 1]
    if bounds is None:
        m = 4.0
        bounds = (tx.min() - m, tx.max() + m, ty.min() - m, ty.max() + m)
    x0, x1, y0, y1 = bounds
    xs = np.arange(x0, x1 + 1e-9, res)
    ys = np.arange(y0, y1 + 1e-9, res)
    XX, YY = np.meshgrid(xs, ys)
    grid = np.column_stack([XX.ravel(), YY.ravel()])
    field = soft_visibility(grid, trees[:, :2], r_vis).reshape(XX.shape)

    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(
        field, origin="lower", extent=(x0, x1, y0, y1),
        cmap="viridis", aspect="equal", interpolation="bilinear",
    )
    ax.scatter(tx, ty, s=28, facecolors="none", edgecolors="white",
               linewidths=1.1, label="ground-truth trunks")
    cb = fig.colorbar(im, ax=ax, shrink=0.85)
    cb.set_label(r"visibility  $v(\mathbf{x})$  (expected # visible trunks)")
    ax.set_title(f"{scenario}\ntrue visibility field  ($r_{{vis}}={r_vis:.2f}$ m)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------
# Comandos
# --------------------------------------------------------------------------
def cmd_build(_args):
    os.makedirs(OUT_DIR, exist_ok=True)

    # r_vis é uma propriedade do sensor+detector -> ajusta GLOBAL (todos os
    # cenários juntos), mais robusto; também reporta o ajuste por cenário.
    pooled = []
    per_scenario_fit = {}
    for sc in SCENARIOS:
        trees = load_ground_truth(sc)
        obs = pool_observations(sc)
        if obs.size:
            per_scenario_fit[sc] = fit_visibility_radius(obs, trees[:, :2])
            pooled.append((obs, trees[:, :2]))

    # ajuste global: soma dos RMSE sobre todos os cenários
    def global_rmse(r):
        tot, n = 0.0, 0
        for obs, tj in pooled:
            pred = count_within(obs[:, :2], tj, r)
            tot += float(np.sum((pred - obs[:, 2]) ** 2))
            n += len(obs)
        return math.sqrt(tot / max(n, 1))

    r_global = float(min(R_GRID, key=global_rmse))
    print("=" * 68)
    print(f"CALIBRAÇÃO de r_vis (limites do config: [{SELF_FILTER_M}, {LIDAR_MAX_RANGE_M}] m)")
    print(f"  r_vis GLOBAL = {r_global:.2f} m   (RMSE global = {global_rmse(r_global):.3f})")
    for sc, fit in per_scenario_fit.items():
        print(f"  {sc:28s} r*={fit['r_vis']:.2f}m  rmse={fit['rmse']:.3f}  "
              f"bias={fit['bias']:+.3f}  corr={fit['corr']:.3f}")
    print("=" * 68)

    # gera v* + imagem + .npz por cenário (usando r_vis global)
    for sc in SCENARIOS:
        trees = load_ground_truth(sc)
        img = os.path.join(OUT_DIR, f"{sc}_true_visibility.png")
        save_ground_truth_image(sc, trees, r_global, img)
        np.savez(
            os.path.join(OUT_DIR, f"{sc}_true_visibility.npz"),
            trees=trees, r_vis=r_global,
        )
        print(f"  salvo: {img}")
    print(f"\nImagens e .npz em: {OUT_DIR}")
    print(f"r_vis usado = {r_global:.2f} m")
    return r_global


# --------------------------------------------------------------------------
# Replay OFFLINE do GP (para comparar AIPP x lawnmower no MESMO GP).
# Lawnmower não grava gp_grid.csv, mas grava point_quality.csv (x,y,score);
# reprocessamos essas medições pelo mesmo ExplorationGP e obtemos mu, sigma.
# --------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from informative_planner.exploration_gp import ExplorationGP
    from informative_planner import config as ipp_config
    _HAVE_GP = True
except Exception as exc:  # pragma: no cover
    _HAVE_GP = False
    _GP_IMPORT_ERR = exc


def _reference_grid(scenario: str):
    """Grade (xy) de um gp_grid.csv informative do cenário (ordem preservada)."""
    for gp in _runs(scenario, "gp_grid.csv"):
        if "/informative/" in gp:
            xy, _, _ = load_gp_grid(gp)
            return xy
    return None


def _replay_gp_object(point_quality_csv: str, grid_xy: np.ndarray):
    """Reprocessa (x,y,point_score) pelo ExplorationGP; retorna o objeto GP."""
    gp = ExplorationGP(
        candidates_xy=grid_xy,
        length_scale=ipp_config.GP_ELL,
        signal_std=ipp_config.GP_SIGMA_S,
        noise_std=ipp_config.GP_SIGMA_N,
        beta=ipp_config.GP_BETA,
        observation_model=ipp_config.GP_OBSERVATION_MODEL,
        observation_footprint_radius=ipp_config.GP_OBSERVATION_FOOTPRINT_RADIUS,
        observation_footprint_sigma=ipp_config.GP_OBSERVATION_FOOTPRINT_SIGMA,
    )
    last = None
    with open(point_quality_csv) as fh:
        for row in csv.DictReader(fh):
            pos = np.array([float(row["x"]), float(row["y"])])
            if last is not None and np.allclose(pos, last):
                continue  # mesma dedup por posição do gaussian_feeder
            last = pos
            gp.add_observation(pos, float(row["point_score"]))
    return gp


def replay_gp(point_quality_csv: str, grid_xy: np.ndarray):
    """Reprocessa e retorna (mu, sigma) na grade."""
    gp = _replay_gp_object(point_quality_csv, grid_xy)
    return gp.mean_.copy(), np.sqrt(np.maximum(np.diag(gp.cov_), 0.0))


def _pick_run(scenario: str, method: str):
    """point_quality.csv mais recente do método (por mtime)."""
    cands = [p for p in _runs(scenario, "point_quality.csv") if f"/{method}/" in p]
    return max(cands, key=os.path.getmtime) if cands else None


def _load_trajectory(point_quality_csv: str):
    traj = os.path.join(os.path.dirname(point_quality_csv), "trajectory.csv")
    if not os.path.exists(traj):
        return None
    xy = []
    with open(traj) as fh:
        for row in csv.DictReader(fh):
            try:
                xy.append([float(row["x"]), float(row["y"])])
            except (KeyError, ValueError):
                pass
    return np.asarray(xy) if xy else None


def cmd_compare_maps(args):
    """Figura: ground truth v* | mapa GP do IPP | mapa GP do lawnmower."""
    if not _HAVE_GP:
        print("ERRO importando ExplorationGP:", _GP_IMPORT_ERR)
        return
    scenario = getattr(args, "scenario", None) or "forest_clustered_10min"
    r_vis = float(np.load(os.path.join(
        OUT_DIR, f"{scenario}_true_visibility.npz"))["r_vis"])
    grid = _reference_grid(scenario)
    trees = load_ground_truth(scenario)
    pq_i = _pick_run(scenario, "informative")
    pq_l = _pick_run(scenario, "lawnmower")
    if grid is None or pq_i is None or pq_l is None:
        print(f"Faltam dados para {scenario} (grid/informative/lawnmower).")
        return

    gp_i = _replay_gp_object(pq_i, grid)
    gp_l = _replay_gp_object(pq_l, grid)
    vstar_c = true_field(grid, trees[:, :2], r_vis)
    sig_i = np.sqrt(np.maximum(np.diag(gp_i.cov_), 0.0))
    sig_l = np.sqrt(np.maximum(np.diag(gp_l.cov_), 0.0))
    ri, mi = rmse(gp_i.mean_, vstar_c), mll(gp_i.mean_, sig_i, vstar_c)
    rl, ml = rmse(gp_l.mean_, vstar_c), mll(gp_l.mean_, sig_l, vstar_c)

    panels = [
        ("Ground truth  $v^*$", vstar_c, None),
        (f"IPP GP map\nRMSE={ri:.2f}   MLL(med.-based)={mi:.1f}", gp_i.mean_,
         _load_trajectory(pq_i)),
        (f"Lawnmower GP map\nRMSE={rl:.2f}   MLL={ml:.1f}", gp_l.mean_,
         _load_trajectory(pq_l)),
    ]
    # escala robusta (98º pct) para um pico espúrio não achatar os outros mapas
    vmax = float(np.percentile(
        np.concatenate([vstar_c, gp_i.mean_, gp_l.mean_]), 98))
    vmax = max(vmax, float(np.max(vstar_c)))
    tx, ty = trees[:, 0], trees[:, 1]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.3))
    im = None
    for ax, (title, vals, traj) in zip(axes, panels):
        xs, ys, Z = _reshape(grid, vals)
        ext = (xs.min(), xs.max(), ys.min(), ys.max())
        im = ax.imshow(Z, origin="lower", extent=ext,
                       cmap="viridis", vmin=0, vmax=vmax, aspect="equal",
                       interpolation="bilinear")
        ax.scatter(tx, ty, s=16, facecolors="none", edgecolors="white",
                   linewidths=0.8)
        if traj is not None and len(traj):
            ax.plot(traj[:, 0], traj[:, 1], color="red", lw=0.7, alpha=0.75,
                    label="UAV path")
            ax.legend(loc="upper right", fontsize=7, framealpha=0.85)
        ax.set_xlim(xs.min(), xs.max())
        ax.set_ylim(ys.min(), ys.max())
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.82,
                 label="visibility  (expected # trunks)")
    fig.suptitle(f"{scenario}: learned visibility map — IPP vs. lawnmower "
                 f"(same budget, same GP; $r_{{vis}}={r_vis:.1f}$ m)",
                 fontsize=13)
    out = os.path.join(OUT_DIR, f"{scenario}_map_comparison.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"salvo: {out}")
    print(f"  IPP : RMSE={ri:.3f}  MLL={mi:.2f}   [{os.path.relpath(pq_i, RESULTS)}]")
    print(f"  Lawn: RMSE={rl:.3f}  MLL={ml:.2f}   [{os.path.relpath(pq_l, RESULTS)}]")


def cmd_replay_eval(_args):
    """RMSE/MLL vs v* com GP reprocessado offline: AIPP x lawnmower, mesmo GP."""
    if not _HAVE_GP:
        print("ERRO importando ExplorationGP:", _GP_IMPORT_ERR)
        return
    r_vis = None
    for sc in SCENARIOS:
        npz = os.path.join(OUT_DIR, f"{sc}_true_visibility.npz")
        if os.path.exists(npz):
            r_vis = float(np.load(npz)["r_vis"]); break
    if r_vis is None:
        r_vis = cmd_build(_args)

    print("=" * 74)
    print(f"RMSE / MLL vs v* — GP REPROCESSADO offline (r_vis={r_vis:.2f} m), "
          f"grade e hiperparâmetros idênticos para os dois métodos")
    print("=" * 74)
    for sc in SCENARIOS:
        grid = _reference_grid(sc)
        if grid is None:
            print(f"\n{sc}: (sem grade de referência informative — pulado)")
            continue
        trees = load_ground_truth(sc)
        vstar = true_field(grid, trees[:, :2], r_vis)
        by_method = defaultdict(list)
        for pq in _runs(sc, "point_quality.csv"):
            method = "informative" if "/informative/" in pq else (
                "lawnmower" if "/lawnmower/" in pq else "other")
            try:
                mu, sigma = replay_gp(pq, grid)
            except Exception:
                continue
            by_method[method].append((rmse(mu, vstar), mll(mu, sigma, vstar)))
        print(f"\n{sc}  (grid {len(grid)} cells):")
        for method, vals in sorted(by_method.items()):
            a = np.asarray(vals)
            print(f"  {method:12s} n={len(a):2d}  "
                  f"RMSE={a[:,0].mean():.3f}±{a[:,0].std():.3f}  "
                  f"MLL(med)={np.median(a[:,1]):.3f}  "
                  f"MLL(mean)={a[:,1].mean():.3f}")


def cmd_eval(_args):
    """RMSE/MLL do GP de cada run vs v*, agregado por método e cenário."""
    # recupera r_vis salvo (ou recalibra)
    r_vis = None
    for sc in SCENARIOS:
        npz = os.path.join(OUT_DIR, f"{sc}_true_visibility.npz")
        if os.path.exists(npz):
            r_vis = float(np.load(npz)["r_vis"])
            break
    if r_vis is None:
        r_vis = cmd_build(_args)

    print("=" * 68)
    print(f"RMSE / MLL do GP vs. campo verdadeiro (r_vis = {r_vis:.2f} m)")
    print("=" * 68)
    for sc in SCENARIOS:
        trees = load_ground_truth(sc)
        by_method = defaultdict(list)
        for gp in _runs(sc, "gp_grid.csv"):
            method = "informative" if "/informative/" in gp else (
                "lawnmower" if "/lawnmower/" in gp else "other")
            xy, mean, std = load_gp_grid(gp)
            vstar = true_field(xy, trees[:, :2], r_vis)
            by_method[method].append((rmse(mean, vstar), mll(mean, std, vstar)))
        if not by_method:
            continue
        print(f"\n{sc}:")
        for method, vals in sorted(by_method.items()):
            arr = np.asarray(vals)
            print(f"  {method:12s} n={len(arr):2d}  "
                  f"RMSE={arr[:,0].mean():.3f}±{arr[:,0].std():.3f}  "
                  f"MLL={arr[:,1].mean():.3f}±{arr[:,1].std():.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("build", help="calibra r_vis, gera v* + imagens + .npz")
    sub.add_parser("eval", help="RMSE/MLL do gp_grid.csv logado vs v* (só informative)")
    sub.add_parser("replay-eval", help="RMSE/MLL com GP reprocessado offline (AIPP x lawnmower)")
    p_cmp = sub.add_parser("compare-maps", help="figura: v* | mapa IPP | mapa lawnmower")
    p_cmp.add_argument("--scenario", default="forest_clustered_10min")
    args = ap.parse_args()
    {"build": cmd_build, "eval": cmd_eval,
     "replay-eval": cmd_replay_eval,
     "compare-maps": cmd_compare_maps}[args.cmd](args)


if __name__ == "__main__":
    main()
