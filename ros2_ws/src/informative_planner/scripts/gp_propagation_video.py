#!/usr/bin/env python3
"""
gp_propagation_video.py
-----------------------
Gera um vídeo (MP4) mostrando a evolução contínua da média posterior (μ)
e da incerteza (σ) do ExplorationGP.

Dois modos:

1. REPLAY (usado pelo benchmark): reproduz as observações reais de um run
   a partir do observations.csv salvo pelo gaussian_feeder.

     python3 gp_propagation_video.py \\
         --observations <run_dir>/gaussian_feeder/observations.csv \\
         --bounds -16 16 -16 16 \\
         --trees-csv <ground_truth.csv> \\
         --out <run_dir>/gp_propagation.mp4

2. DEMO (sem argumentos): missão greedy-UCB simulada sobre o ground truth
   de small_clustered — útil para tunar GP_ELL visualmente (--ell N).

No replay, os campos são reconstruídos pela MESMA lógica usada online pelo
planner: ExplorationGP.add_observation() sobre a grade de candidatos, com o
ObservationModel configurado (H_naive ou footprint). Assim a incerteza do
vídeo é sqrt(diag(cov_)), igual à que entra no UCB/fantasy/RRT.

Entre keyframes consecutivos os campos são interpolados com easing
suave, produzindo propagação contínua no espaço e no tempo.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
SRC_DIR = PKG_DIR.parent
WORKSPACE_DIR = SRC_DIR.parent

sys.path.insert(0, str(PKG_DIR))

from informative_planner import config  # noqa: E402
from informative_planner.exploration_gp import ExplorationGP  # noqa: E402
from informative_planner.utils import generate_waypoint_candidates  # noqa: E402

DEFAULT_NPZ = (
    SRC_DIR
    / "tree_measuring"
    / "groundtruths"
    / "small_clustered_visible_tree_groundtruth.npz"
)

# Estética (tema escuro único, colormaps perceptualmente uniformes)
BG = "#101418"
PANEL_BG = "#161b21"
INK = "#e8ecef"
INK_MUTED = "#9aa4ad"
GRID_COLOR = "#2a3138"
PATH_COLOR = "#e8ecef"
DRONE_COLOR = "#ffffff"
CMAP_MEAN = "viridis"
CMAP_STD = "magma"


# ---------------------------------------------------------------------------
# Posterior Kalman incremental (mesma lógica do planner online)
# ---------------------------------------------------------------------------

def kalman_grid_axes(gp: ExplorationGP) -> tuple[np.ndarray, np.ndarray]:
    """Return sorted x/y axes of the fixed Kalman candidate grid."""
    grid_xy = gp.X_grid
    return np.unique(grid_xy[:, 0]), np.unique(grid_xy[:, 1])


def kalman_grid_snapshot(
    gp: ExplorationGP,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return flattened mean/std fields directly from gp.mean_ and gp.cov_.

    This is intentionally the same representation used by gaussian_feeder's
    Kalman map and by the planner's UCB/fantasy state.
    """
    mean_grid = np.full((len(grid_y), len(grid_x)), np.nan)
    std_grid = np.full_like(mean_grid, np.nan)
    std_values = np.sqrt(np.maximum(np.diag(gp.cov_), 0.0))

    x_to_idx = {float(x): i for i, x in enumerate(grid_x)}
    y_to_idx = {float(y): i for i, y in enumerate(grid_y)}
    for pos, mean, std in zip(gp.X_grid, gp.mean_, std_values):
        x_idx = x_to_idx[float(pos[0])]
        y_idx = y_to_idx[float(pos[1])]
        mean_grid[y_idx, x_idx] = mean
        std_grid[y_idx, x_idx] = std

    return np.nan_to_num(mean_grid).ravel(), np.nan_to_num(std_grid).ravel()


def compute_keyframe_fields(
    gp: ExplorationGP,
    obs_xy: np.ndarray,
    obs_y: np.ndarray,
    keyframes: list[int],
    grid_x: np.ndarray,
    grid_y: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Replay observations through ExplorationGP.add_observation().

    Because add_observation() builds H through gp.observation_model, this is
    the same uncertainty propagation used by the online planner.
    """
    results: list[tuple[np.ndarray, np.ndarray]] = []
    wanted = iter(sorted(set(keyframes)))
    next_kf = next(wanted, None)

    if next_kf == 0:
        results.append(kalman_grid_snapshot(gp, grid_x, grid_y))
        next_kf = next(wanted, None)

    for index, (pos, value) in enumerate(zip(obs_xy, obs_y), start=1):
        gp.add_observation(pos, float(value))
        while next_kf is not None and index >= next_kf:
            results.append(kalman_grid_snapshot(gp, grid_x, grid_y))
            next_kf = next(wanted, None)

    return results


def make_exploration_gp(
    candidates_xy: np.ndarray,
    ell: float,
    beta: float,
) -> ExplorationGP:
    """Create the same ExplorationGP configuration used by the planner."""
    return ExplorationGP(
        candidates_xy=candidates_xy,
        length_scale=ell,
        signal_std=config.GP_SIGMA_S,
        noise_std=config.GP_SIGMA_N,
        beta=beta,
        observation_model=config.GP_OBSERVATION_MODEL,
        observation_footprint_radius=config.GP_OBSERVATION_FOOTPRINT_RADIUS,
        observation_footprint_sigma=config.GP_OBSERVATION_FOOTPRINT_SIGMA,
    )


def observation_ping_radius(gp: ExplorationGP) -> float:
    """Radius shown around each observation in the video overlay."""
    model = gp.observation_model
    if model.mode == model.FOOTPRINT:
        return float(model.footprint_radius)
    return float(config.WAYPOINT_GRID_RESOLUTION)


# ---------------------------------------------------------------------------
# Fontes de observações
# ---------------------------------------------------------------------------

def load_observations_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Lê observations.csv do gaussian_feeder (colunas x, y, point_score)."""
    xs, ys, scores = [], [], []
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["point_score"])
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                xs.append(x)
                ys.append(y)
                scores.append(z)
    return np.column_stack((xs, ys)), np.asarray(scores)


def load_trees_csv(path: Path) -> np.ndarray:
    """Lê posições de árvores de um CSV com colunas x e y (ground truth)."""
    pts = []
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                pts.append((float(row["x"]), float(row["y"])))
            except (KeyError, ValueError):
                continue
    return np.asarray(pts) if pts else np.empty((0, 2))


def bilinear_sample(xs: np.ndarray, ys: np.ndarray, field: np.ndarray,
                    x: float, y: float) -> float:
    """Amostra bilinear do campo (ys, xs) em uma posição contínua."""
    ix = np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2)
    iy = np.clip(np.searchsorted(ys, y) - 1, 0, len(ys) - 2)
    tx = np.clip((x - xs[ix]) / (xs[ix + 1] - xs[ix]), 0.0, 1.0)
    ty = np.clip((y - ys[iy]) / (ys[iy + 1] - ys[iy]), 0.0, 1.0)
    f00, f01 = field[iy, ix], field[iy, ix + 1]
    f10, f11 = field[iy + 1, ix], field[iy + 1, ix + 1]
    return float(
        f00 * (1 - tx) * (1 - ty)
        + f01 * tx * (1 - ty)
        + f10 * (1 - tx) * ty
        + f11 * tx * ty
    )


def simulate_demo_mission(
    gp: ExplorationGP,
    xs: np.ndarray,
    ys: np.ndarray,
    truth: np.ndarray,
    n_obs: int,
    step: float,
    arrival_tol: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Missão greedy-UCB simulada: persegue o candidato de maior UCB em
    passos curtos, observando o ground truth a cada passo.
    """
    bounds = (xs[0], xs[-1], ys[0], ys[-1])
    pos = np.array([bounds[0] + 2.0, bounds[2] + 2.0])
    target = None
    obs_xy, obs_y = [], []

    for _ in range(n_obs):
        if target is None or np.linalg.norm(target - pos) < arrival_tol:
            ucb = gp.ucb_grid()
            target = gp.X_grid[int(np.argmax(ucb))].copy()

        direction = target - pos
        dist = np.linalg.norm(direction)
        if dist > 1e-6:
            pos = pos + direction / dist * min(step, dist)
        pos[0] = np.clip(pos[0], bounds[0], bounds[1])
        pos[1] = np.clip(pos[1], bounds[2], bounds[3])

        z = bilinear_sample(xs, ys, truth, pos[0], pos[1])
        z = max(z + rng.normal(0.0, gp.noise_var ** 0.5), 0.0)
        gp.add_observation(pos, z)
        obs_xy.append(pos.copy())
        obs_y.append(z)

    return np.asarray(obs_xy), np.asarray(obs_y)


# ---------------------------------------------------------------------------
# Renderização
# ---------------------------------------------------------------------------

def smoothstep(t: float) -> float:
    """Easing suave (Hermite) para a interpolação temporal dos campos."""
    return t * t * (3.0 - 2.0 * t)


def render_video(
    out_path: Path,
    obs_xy: np.ndarray,
    keyframes: list[int],
    fields: list[tuple[np.ndarray, np.ndarray]],
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    tree_xy: np.ndarray,
    ell: float,
    sf: float,
    noise_std: float,
    beta: float,
    mean_vmax: float,
    fps: int,
    frames_per_key: int,
    hold_intro: int,
    hold_final: int,
    title: str,
    observation_label: str,
    ping_radius: float,
) -> None:
    """Renderiza os frames com matplotlib e grava MP4 via OpenCV."""
    import cv2

    extent = (grid_x[0], grid_x[-1], grid_y[0], grid_y[-1])
    shape = (len(grid_y), len(grid_x))

    fig, axes = plt.subplots(
        1, 2, figsize=(12.8, 6.4), dpi=100, facecolor=BG,
        gridspec_kw={"wspace": 0.12},
    )
    fig.subplots_adjust(left=0.05, right=0.97, top=0.86, bottom=0.10)

    panels = []
    specs = [
        ("Média posterior  μ(x)", CMAP_MEAN, 0.0, mean_vmax, "point score"),
        ("Incerteza  σ(x)", CMAP_STD, 0.0, sf, "desvio padrão"),
    ]
    for ax, (panel_title, cmap, vmin, vmax, cbar_label) in zip(axes, specs):
        ax.set_facecolor(PANEL_BG)
        im = ax.imshow(
            np.zeros(shape),
            origin="lower", extent=extent, cmap=cmap,
            vmin=vmin, vmax=vmax, interpolation="bilinear",
        )
        ax.set_title(panel_title, color=INK, fontsize=13, pad=10)
        ax.set_xlabel("x [m]", color=INK_MUTED, fontsize=9)
        ax.set_ylabel("y [m]", color=INK_MUTED, fontsize=9)
        ax.tick_params(colors=INK_MUTED, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(GRID_COLOR)
        ax.set_aspect("equal", adjustable="box")

        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
        cbar.set_label(cbar_label, color=INK_MUTED, fontsize=8)
        cbar.ax.tick_params(colors=INK_MUTED, labelsize=7)
        cbar.outline.set_edgecolor(GRID_COLOR)

        if len(tree_xy):
            ax.scatter(
                tree_xy[:, 0], tree_xy[:, 1], marker="x", s=28,
                c=INK, linewidths=1.1, alpha=0.85, zorder=5,
            )
        (path_line,) = ax.plot(
            [], [], color=PATH_COLOR, lw=1.1, alpha=0.35, zorder=6,
        )
        (drone_dot,) = ax.plot(
            [], [], marker="o", ms=7, mfc=DRONE_COLOR, mec=BG,
            mew=1.2, ls="", zorder=8,
        )
        ping = plt.Circle(
            (0, 0), 0.0, fill=False, color=DRONE_COLOR, lw=1.4,
            alpha=0.0, zorder=7,
        )
        ax.add_patch(ping)
        panels.append({
            "im": im, "path": path_line, "drone": drone_dot, "ping": ping,
        })

    fig.suptitle(title, color=INK, fontsize=15, y=0.97)
    fig.text(
        0.05, 0.905,
        f"ℓ = {ell:.1f} m    sf = {sf:.1f}    "
        f"σₙ = {noise_std:.2f}    β = {beta:.1f}    "
        f"H = {observation_label}",
        color=INK_MUTED, fontsize=10,
    )
    status_text = fig.text(
        0.97, 0.905, "", color=INK, fontsize=10, ha="right",
    )

    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    w -= w % 2
    h -= h % 2
    writer = cv2.VideoWriter(
        str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Não foi possível abrir o writer em {out_path}")

    def grab_frame() -> None:
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.write(cv2.cvtColor(buf[:h, :w], cv2.COLOR_RGB2BGR))

    def draw_state(mean_f, std_f, path_upto, pos, n_obs, ping_t):
        for k, field in zip(panels, (mean_f, std_f)):
            k["im"].set_data(field.reshape(shape))
            pts = obs_xy[:max(path_upto, 1)]
            k["path"].set_data(pts[:, 0], pts[:, 1])
            k["drone"].set_data([pos[0]], [pos[1]])
            k["ping"].set_center((pos[0], pos[1]))
            k["ping"].set_radius(float(ping_t) * ping_radius)
            k["ping"].set_alpha(0.55 * (1.0 - float(ping_t)))
        status_text.set_text(f"obs: {n_obs:4d} / {len(obs_xy)}")

    # Prior
    m_prev, s_prev = fields[0]
    for _ in range(hold_intro):
        draw_state(m_prev, s_prev, 0, obs_xy[0], 0, 1.0)
        grab_frame()

    # Transições entre keyframes consecutivos
    n_key = len(keyframes) - 1   # keyframes[0] == 0 (prior)
    for i in range(1, len(keyframes)):
        m0, s0 = fields[i - 1]
        m1, s1 = fields[i]
        k0, k1 = keyframes[i - 1], keyframes[i]
        p0 = obs_xy[max(k0 - 1, 0)]
        p1 = obs_xy[k1 - 1]

        for f in range(frames_per_key):
            t = smoothstep((f + 1) / frames_per_key)
            mean_f = (1 - t) * m0 + t * m1
            std_f = (1 - t) * s0 + t * s1
            # Caminho e drone avançam junto com a interpolação
            k_now = k0 + (k1 - k0) * (f + 1) // frames_per_key
            pos = (1 - t) * p0 + t * p1
            draw_state(mean_f, std_f, k_now, pos, k_now, t)
            grab_frame()

        if i % 40 == 0 or i == n_key:
            print(f"  frames: keyframe {i}/{n_key}")

    # Segura o posterior final
    m_end, s_end = fields[-1]
    for _ in range(hold_final):
        draw_state(m_end, s_end, len(obs_xy), obs_xy[-1], len(obs_xy), 1.0)
        grab_frame()

    writer.release()
    plt.close(fig)


# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--observations", type=Path, default=None,
                        help="observations.csv de um run real (modo replay)")
    parser.add_argument("--bounds", type=float, nargs=4, default=None,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
                        help="limites do mapa; obrigatório no modo replay")
    parser.add_argument("--trees-csv", type=Path, default=None,
                        help="CSV de ground truth com colunas x,y (opcional)")
    parser.add_argument("--out", type=Path,
                        default=WORKSPACE_DIR / "gp_propagation.mp4")
    parser.add_argument("--ell", type=float, default=config.GP_ELL)
    parser.add_argument("--beta", type=float, default=config.GP_BETA)
    parser.add_argument("--grid-res", type=float, default=0.5,
                        help=("mantido por compatibilidade; o replay usa a "
                              "grade de candidatos do planner"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--target-duration", type=float, default=30.0,
                        help="duração alvo do vídeo [s]")
    parser.add_argument("--max-keyframes", type=int, default=0,
                        help=("limite de snapshots; 0 salva um estado por "
                              "observação, sem downsample"))
    # Opções do modo demo
    parser.add_argument("--npz", type=Path, default=DEFAULT_NPZ)
    parser.add_argument("--n-obs", type=int, default=90)
    parser.add_argument("--step", type=float, default=2.5)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    tree_xy = np.empty((0, 2))

    if args.observations is not None:
        # ── Modo replay: observações reais de um run do benchmark ─────────
        obs_xy, obs_y = load_observations_csv(args.observations)
        if len(obs_xy) < 2:
            raise SystemExit(
                f"Poucas observações em {args.observations} "
                f"({len(obs_xy)}); vídeo não gerado."
            )
        if args.bounds is not None:
            x_min, x_max, y_min, y_max = args.bounds
        else:
            margin = 2.0
            x_min, x_max = obs_xy[:, 0].min() - margin, obs_xy[:, 0].max() + margin
            y_min, y_max = obs_xy[:, 1].min() - margin, obs_xy[:, 1].max() + margin
        if args.trees_csv is not None and args.trees_csv.exists():
            tree_xy = load_trees_csv(args.trees_csv)
        title = "Propagação do GP Kalman — replay do planner"
        bounds = (x_min, x_max, y_min, y_max)
        candidates_xy = generate_waypoint_candidates(bounds)[:, :2]
        gp = make_exploration_gp(candidates_xy, args.ell, args.beta)
    else:
        # ── Modo demo: missão simulada sobre o ground truth ───────────────
        data = np.load(args.npz, allow_pickle=True)
        gt_xs, gt_ys = data["xs"], data["ys"]
        truth = np.nan_to_num(data["scores"], nan=0.0)
        tree_xy = data["tree_xy"]
        sample_xy = data["sample_xy"]

        # O npz pode ter candidatos além dos bounds atuais (grade antiga);
        # sem o filtro o drone persegue alvos inalcançáveis na borda.
        inside = (
            (sample_xy[:, 0] >= gt_xs[0]) & (sample_xy[:, 0] <= gt_xs[-1])
            & (sample_xy[:, 1] >= gt_ys[0]) & (sample_xy[:, 1] <= gt_ys[-1])
        )
        sample_xy = sample_xy[inside]

        policy_gp = make_exploration_gp(sample_xy, args.ell, args.beta)
        print(f"Simulando missão demo: {args.n_obs} observações, "
              f"ℓ={args.ell:.1f} m, β={args.beta:.1f}")
        rng = np.random.default_rng(args.seed)
        obs_xy, obs_y = simulate_demo_mission(
            policy_gp, gt_xs, gt_ys, truth,
            n_obs=args.n_obs, step=args.step, arrival_tol=1.5, rng=rng,
        )
        x_min, x_max = gt_xs[0], gt_xs[-1]
        y_min, y_max = gt_ys[0], gt_ys[-1]
        gp = make_exploration_gp(sample_xy, args.ell, args.beta)
        title = "Propagação do GP Matérn 3/2 — missão informativa (UCB)"

    n = len(obs_xy)
    grid_x, grid_y = kalman_grid_axes(gp)

    # Keyframes: prefixos de observações onde o posterior do planner é salvo
    if args.max_keyframes and args.max_keyframes > 0:
        stride = max(1, int(np.ceil(n / args.max_keyframes)))
    else:
        stride = 1
    keyframes = [0] + list(range(stride, n, stride)) + [n]
    keyframes = sorted(set(keyframes))

    total_frames = args.target_duration * args.fps
    frames_per_key = int(np.clip(
        round(total_frames / max(len(keyframes) - 1, 1)), 1, 12,
    ))

    print(f"Observações: {n} | keyframes: {len(keyframes) - 1} "
          f"(stride {stride}) | frames/keyframe: {frames_per_key} "
          f"| grade planner: {len(grid_x)}x{len(grid_y)} "
          f"| H={gp.observation_model.describe()}")

    fields = compute_keyframe_fields(
        gp, obs_xy, obs_y, keyframes, grid_x, grid_y,
    )

    mean_vmax = max(1.0, float(np.percentile(obs_y, 99)))

    print(f"Renderizando vídeo em {args.out} ...")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    render_video(
        out_path=args.out,
        obs_xy=obs_xy,
        keyframes=keyframes,
        fields=fields,
        grid_x=grid_x,
        grid_y=grid_y,
        tree_xy=tree_xy,
        ell=args.ell,
        sf=gp.sf,
        noise_std=float(np.sqrt(gp.noise_var)),
        beta=gp.beta,
        mean_vmax=mean_vmax,
        fps=args.fps,
        frames_per_key=frames_per_key,
        hold_intro=int(0.7 * args.fps),
        hold_final=int(1.5 * args.fps),
        title=title,
        observation_label=gp.observation_model.describe(),
        ping_radius=observation_ping_radius(gp),
    )
    n_frames = ((len(keyframes) - 1) * frames_per_key
                + int(0.7 * args.fps) + int(1.5 * args.fps))
    print(f"Pronto: {args.out}  (~{n_frames / args.fps:.1f} s @ {args.fps} fps)")


if __name__ == "__main__":
    main()
