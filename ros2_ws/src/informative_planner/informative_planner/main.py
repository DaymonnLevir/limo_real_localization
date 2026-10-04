"""
main.py
-------
Simulação de uma missão de mapeamento de DAP com drone.

O fluxo principal é:
  1. cria uma floresta sintética;
  2. inicializa o mapa de árvores;
  3. escolhe waypoints por UCB do GP de exploração + bônus angular analítico;
  4. mede árvores visíveis com LiDAR stub (RANSAC);
  5. acumula medições de DAP por árvore (incerteza = cobertura angular);
  6. gera gráficos e vídeo da exploração.
"""

import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

import config
from drone_stub import DroneStub
from exploration_gp import ExplorationGP
from lidar_stub import LiDARStub
from planner import (
    compute_angular_bonus,
    compute_visit_utility,
    find_next_waypoint_ucb,
    generate_waypoint_candidates,
)
from tree import Tree
from tree_map import TreeMap


SCENARIOS = ("random", "clusters", "groves")


# ---------------------------------------------------------------------------
# 1. CRIA A FLORESTA (simulada)
# ---------------------------------------------------------------------------

def create_forest(n_trees: int = 25,
                  area: float = 30.0,
                  seed: int = 7) -> list:
    """Cria uma floresta sintética aleatória."""
    rng = np.random.default_rng(seed)

    positions = []
    attempts = 0
    while len(positions) < n_trees and attempts < 10_000:
        x = rng.uniform(-area / 2, area / 2)
        y = rng.uniform(-area / 2, area / 2)
        too_close = any(
            np.linalg.norm(np.array([x, y]) - np.array(p)) < 2.0
            for p in positions
        )
        if not too_close:
            positions.append([x, y])
        attempts += 1

    true_daps = rng.lognormal(mean=np.log(0.22), sigma=0.3, size=len(positions))
    true_daps = np.clip(true_daps, 0.05, 0.60)

    return [
        Tree(tree_id=i, x=float(x), y=float(y), true_dap=float(dap))
        for i, ((x, y), dap) in enumerate(zip(positions, true_daps))
    ]


def create_clustered_forest(cluster_specs: list,
                            seed: int,
                            min_spacing: float = 1.1) -> list:
    """
    Cria uma floresta com manchas densas separadas.

    cluster_specs: (centro_x, centro_y, quantidade, dispersao, dap_medio)
    """
    rng = np.random.default_rng(seed)
    positions = []
    daps = []

    for cx, cy, count, spread, mean_dap in cluster_specs:
        added = 0
        attempts = 0
        while added < count and attempts < 10_000:
            candidate = rng.normal([cx, cy], spread, size=2)
            too_close = any(
                np.linalg.norm(candidate - np.array(existing)) < min_spacing
                for existing in positions
            )
            inside_bounds = (
                -15.0 <= candidate[0] <= 15.0
                and -15.0 <= candidate[1] <= 15.0
            )

            if inside_bounds and not too_close:
                positions.append(candidate.tolist())
                dap = rng.lognormal(mean=np.log(mean_dap), sigma=0.22)
                daps.append(float(np.clip(dap, 0.05, 0.60)))
                added += 1

            attempts += 1

        if added < count:
            raise RuntimeError("Não foi possível posicionar todas as árvores.")

    return [
        Tree(tree_id=i, x=float(x), y=float(y), true_dap=float(dap))
        for i, ((x, y), dap) in enumerate(zip(positions, daps))
    ]


def create_forest_scenario(scenario: str):
    """Retorna árvores, limites e descrição para um cenário de exemplo."""
    bounds = (-15.0, 15.0, -15.0, 15.0)

    if scenario == "random":
        return (
            create_forest(n_trees=25, area=30.0, seed=7),
            bounds,
            "árvores distribuídas aleatoriamente",
        )

    if scenario == "clusters":
        cluster_specs = [
            (-9.0, -8.0, 9, 1.9, 0.20),
            (7.5, -6.5, 8, 1.8, 0.25),
            (0.5, 8.0, 8, 2.0, 0.30),
        ]
        return (
            create_clustered_forest(cluster_specs, seed=11),
            bounds,
            "três concentrações densas e separadas",
        )

    if scenario == "groves":
        cluster_specs = [
            (-10.0, 9.0, 6, 1.5, 0.18),
            (9.0, 8.0, 7, 1.7, 0.24),
            (-9.0, -8.5, 6, 1.6, 0.28),
            (8.0, -8.0, 6, 1.5, 0.21),
        ]
        return (
            create_clustered_forest(cluster_specs, seed=17),
            bounds,
            "quatro talhões compactos separados por clareiras",
        )

    raise ValueError(f"Cenário desconhecido: {scenario}")


# ---------------------------------------------------------------------------
# 2. MAPA CONTÍNUO DO POSTERIOR GP DE EXPLORAÇÃO
# ---------------------------------------------------------------------------

def compute_gp_field(
    exploration_gp: ExplorationGP,
    bounds: tuple,
    resolution: float = None,
):
    """
    Calcula o mapa do posterior GP de utilidade de exploração.

    Retorna a média e o desvio padrão do GP para cada célula da grade.
    Sem observações → média = 0, std = signal_std (mapa completamente escuro,
    pois nada foi descoberto ainda). Conforme o drone visita posições, as
    células próximas "acendem" com a utilidade real observada.
    """
    if resolution is None:
        resolution = config.INFO_GAIN_MAP_RESOLUTION

    x_min, x_max, y_min, y_max = bounds
    xs = np.arange(x_min, x_max + 0.5 * resolution, resolution)
    ys = np.arange(y_min, y_max + 0.5 * resolution, resolution)

    XX, YY = np.meshgrid(xs, ys)
    X_test = np.column_stack([XX.ravel(), YY.ravel()])

    mean_flat, std_flat = exploration_gp.predict(X_test)

    gp_mean = mean_flat.reshape(len(ys), len(xs))
    gp_std = std_flat.reshape(len(ys), len(xs))

    return xs, ys, gp_mean, gp_std


def record_gp_frame(history: dict, exploration_gp: ExplorationGP,
                    bounds: tuple, drone_xy: np.ndarray,
                    step: int) -> None:
    """Salva snapshot do posterior GP para montar o vídeo."""
    xs, ys, gp_mean, gp_std = compute_gp_field(exploration_gp, bounds)
    history["gp_frames"].append({
        "step": step,
        "time": history["time"][-1],
        "trace": history["trace"][-1],
        "drone_xy": np.array(drone_xy, dtype=float).copy(),
        "path": np.array(history["waypoints"], dtype=float).copy(),
        "xs": xs,
        "ys": ys,
        "gp_mean": gp_mean,
        "gp_std": gp_std,
    })


# ---------------------------------------------------------------------------
# 3. LOOP PRINCIPAL DA MISSÃO
# ---------------------------------------------------------------------------

def run_mission(trees: list, bounds: tuple = None,
                scenario_name: str = None,
                verbose: bool = True):
    """Executa a missão completa de mapeamento de DAP."""
    tree_map = TreeMap(trees)

    # Posição inicial aleatória (dentro dos limites com margem de segurança)
    if bounds is None:
        bounds = (-15.0, 15.0, -15.0, 15.0)
    x_min, x_max, y_min, y_max = bounds
    margin = 2.0
    rng_start = np.random.default_rng()
    x0 = float(rng_start.uniform(x_min + margin, x_max - margin))
    y0 = float(rng_start.uniform(y_min + margin, y_max - margin))
    start_pos = np.array([x0, y0, config.FLIGHT_HEIGHT])

    drone = DroneStub(initial_pos=start_pos)
    lidar = LiDARStub(seed=42)

    # Grade de candidatos — fixa durante toda a missão; o KF do GP é
    # definido sobre esses pontos (Popovic et al., 2017).
    candidates = generate_waypoint_candidates(bounds)

    # GP de exploração: kernel Matérn 3/2, atualizado por Filtro de Kalman.
    # Mapeia N_vis(x,y) — propriedade ESTÁTICA do espaço.
    # P₀ = K_Matérn + σ_n² I  (prior completamente desconhecido).
    exploration_gp = ExplorationGP(
        candidates_xy=candidates[:, :2],
        length_scale=config.GP_ELL,
        signal_std=config.GP_SIGMA_S,
        noise_std=config.GP_SIGMA_N,
        beta=config.GP_BETA,
        observation_model=config.GP_OBSERVATION_MODEL,
        observation_footprint_radius=config.GP_OBSERVATION_FOOTPRINT_RADIUS,
        observation_footprint_sigma=config.GP_OBSERVATION_FOOTPRINT_SIGMA,
    )

    history = {
        "time": [0.0],
        "trace": [tree_map.compute_total_uncertainty()],
        "waypoints": [start_pos[:2].copy()],
        "bounds": bounds,
        "gp_frames": [],
    }
    record_gp_frame(history, exploration_gp, bounds, drone.xy, step=0)

    if verbose:
        print("=" * 60)
        if scenario_name:
            print(f"CENÁRIO: {scenario_name}")
        print(f"MISSÃO INICIADA — {len(trees)} árvores | "
              f"orçamento {config.TIME_BUDGET}s")
        print(f"Início aleatório: ({x0:.1f}, {y0:.1f})")
        print(f"GP exploração: kernel=Matérn 3/2 (Popovic et al.) | "
              f"ell={config.GP_ELL}m  sf={config.GP_SIGMA_S}  beta={config.GP_BETA}")
        print(f"H: {exploration_gp.observation_model.describe()}")
        print(f"LiDAR: alcance útil={config.LIDAR_MAX_RANGE}m")
        print(f"incerteza inicial = {tree_map.compute_total_uncertainty():.4f} m")
        print("=" * 60)

    step = 0
    while not drone.is_budget_spent():
        step += 1

        # Planeja próximo waypoint: UCB do GP (cobertura) + bônus angular analítico
        waypoint, score = find_next_waypoint_ucb(
            exploration_gp, candidates, drone.xy, trees=tree_map.trees
        )

        drone.fly_to(waypoint)
        visible = tree_map.get_visible_trees(drone.xy)

        # GP aprende apenas N_vis (propriedade estática do espaço)
        utility = compute_visit_utility(visible)
        exploration_gp.add_observation(drone.xy, utility)

        if verbose:
            ang_bonus = compute_angular_bonus(drone.xy, tree_map.trees)
            print(
                f"[t={drone.elapsed_time:5.1f}s | step {step:2d}] "
                f"wp=({waypoint[0]:5.1f},{waypoint[1]:5.1f}) | "
                f"vistas={len(visible):2d} | "
                f"N_vis={utility:.0f}  ang+={ang_bonus:.1f} | "
                f"score={score:.2f} | obs_gp={exploration_gp.n_observations}"
            )

        if visible:
            measurements = lidar.scan(drone.xy, trees, visible)
            tree_map.update(measurements, visible, drone_xy=drone.xy)

        uncertainty_now = tree_map.compute_total_uncertainty()
        history["time"].append(drone.elapsed_time)
        history["trace"].append(uncertainty_now)
        history["waypoints"].append(drone.xy.copy())
        record_gp_frame(history, exploration_gp, bounds, drone.xy, step=step)

    if verbose:
        print("\n" + "=" * 60)
        print(f"MISSÃO ENCERRADA — tempo total: {drone.elapsed_time:.1f}s")
        unc_final = tree_map.compute_total_uncertainty()
        unc_inicial = history["trace"][0]
        print(f"Σσ_DAP: {unc_inicial:.4f} -> {unc_final:.4f} m  "
              f"(redução {100*(1-unc_final/unc_inicial):.1f}%)")
        observed = [t for t in trees if t.num_observations > 0]
        print(f"Árvores observadas: {len(observed)}/{len(trees)}")
        print(f"Observações GP: {exploration_gp.n_observations}")
        print("\nTop 5 árvores com maior incerteza restante:")
        for tid, std in tree_map.most_uncertain_trees(5):
            t = trees[tid]
            print(f"  Árvore {tid:2d}: sigma={std:.4f}m | "
                  f"DAP real={t.true_dap:.3f}m | "
                  f"estimado={t.dap_estimate:.3f}m | "
                  f"obs={t.num_observations} | "
                  f"setores={t.angle_sector_count()}")
        print("=" * 60)

    return tree_map, history, drone


# ---------------------------------------------------------------------------
# 4. VISUALIZAÇÃO
# ---------------------------------------------------------------------------

def draw_gp_map(ax, trees, frame, vmax_mean: float, title: str):
    """
    Desenha um frame do mapa do posterior GP de exploração.

    A média do GP representa a utilidade estimada de cada ponto:
      - Início da missão: tudo escuro (média = 0, mapa desconhecido)
      - Conforme o drone explora: áreas visitadas "acendem" com utilidade real
      - GP interpola para vizinhança das posições visitadas

    Linhas de contorno brancas mostram zonas de alto UCB (próximo passo alvo).
    """
    X, Y = np.meshgrid(frame["xs"], frame["ys"])

    cf = ax.contourf(
        X, Y, frame["gp_mean"],
        levels=24,
        cmap="viridis",
        vmin=0.0,
        vmax=vmax_mean,
    )

    # Contornos brancos = regiões de alto UCB (exploração + exploitation)
    ucb = frame["gp_mean"] + 2.0 * frame["gp_std"]
    ucb_max = float(np.max(ucb))
    if ucb_max > 0.5:
        ucb_levels = np.linspace(0.6 * ucb_max, ucb_max, 4)
        ax.contour(X, Y, ucb, levels=ucb_levels,
                   colors="white", linewidths=0.7, alpha=0.5)

    # Posições das árvores (referência de simulação)
    ax.scatter([t.x for t in trees], [t.y for t in trees],
               c="red", s=18, alpha=0.6, marker="^", label="Árvores (ref.)")

    path = frame["path"]
    if len(path) > 0:
        ax.plot(path[:, 0], path[:, 1], "b-o",
                markersize=4, linewidth=1.4, label="Trajetória")
    ax.plot(frame["drone_xy"][0], frame["drone_xy"][1],
            "w*", markersize=13, label="Drone")

    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(title)
    ax.legend(fontsize=8, loc="upper right")
    return cf


def plot_results(trees: list, tree_map: TreeMap,
                 history: dict, output_path: str = "results.png"):
    """Gera painel estático final."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    frames = history["gp_frames"]
    final_frame = frames[-1]

    # Escala fixa pelo máximo de utilidade observado em todos os frames
    vmax = max(float(np.max(frame["gp_mean"])) for frame in frames)
    if vmax <= 0:
        vmax = 1.0

    cf = draw_gp_map(
        axes[0], trees, final_frame, vmax,
        "Mapa GP — Utilidade de Exploração (final)",
    )
    fig.colorbar(cf, ax=axes[0], label="Utilidade estimada (árvores vistas + ângulos novos)")

    ax = axes[1]
    ax.set_title("Trajetória do Drone")
    for t in trees:
        circle = plt.Circle((t.x, t.y), t.trunk_radius(),
                            color="forestgreen", alpha=0.4)
        ax.add_patch(circle)
        ax.plot(t.x, t.y, "g.", markersize=3)
    waypoints = np.array(history["waypoints"])
    ax.plot(waypoints[:, 0], waypoints[:, 1],
            "b-o", markersize=5, linewidth=1.5, label="Trajetória")
    ax.plot(waypoints[0, 0], waypoints[0, 1],
            "bs", markersize=10, label="Início (aleatório)")
    ax.plot(waypoints[-1, 0], waypoints[-1, 1],
            "r*", markersize=12, label="Fim")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.set_title("Incerteza Total DAP")
    ax.plot(history["time"], history["trace"], "b-o", markersize=4, linewidth=2)
    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel("Σσ_DAP [m]")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close(fig)
    print(f"\nGráficos salvos em: {output_path}")


def render_exploration_video(trees: list, history: dict,
                             output_path: str = "exploration.mp4") -> None:
    """
    Gera MP4 mostrando a evolução do mapa GP ao longo da exploração.

    O vídeo deve mostrar:
      - Início: mapa todo escuro (nada explorado, GP prior = 0)
      - Progressão: regiões visitadas "acendem" conforme o drone descobre árvores
      - Final: mapa com hotspots nas áreas de alta densidade de árvores
    """
    frames = history.get("gp_frames", [])
    if not frames:
        print("Nenhum frame disponível para vídeo.")
        return

    # Escala fixa pelo máximo de utilidade em todos os frames (para comparação)
    vmax = max(float(np.max(frame["gp_mean"])) for frame in frames)
    if vmax <= 0:
        vmax = 1.0

    fig = plt.figure(figsize=(12, 6))
    writer = animation.FFMpegWriter(fps=1.5, bitrate=1800)

    with writer.saving(fig, output_path, dpi=120):
        for frame in frames:
            fig.clf()
            ax_map = fig.add_subplot(1, 2, 1)
            ax_trace = fig.add_subplot(1, 2, 2)

            cf = draw_gp_map(
                ax_map, trees, frame, vmax,
                f"GP Exploração | step {frame['step']} | t={frame['time']:.1f}s",
            )
            fig.colorbar(cf, ax=ax_map, label="Utilidade GP (árvores + ângulos)")

            ax_trace.plot(history["time"], history["trace"],
                          color="lightgray", linewidth=2)
            times = [f["time"] for f in frames
                     if f["time"] <= frame["time"] + 1e-9]
            traces = [f["trace"] for f in frames
                      if f["time"] <= frame["time"] + 1e-9]
            ax_trace.plot(times, traces, "b-o", markersize=4)
            ax_trace.set_xlabel("Tempo [s]")
            ax_trace.set_ylabel("Σσ_DAP [m]")
            ax_trace.set_title("Incerteza Total DAP")
            ax_trace.grid(True, alpha=0.3)

            fig.tight_layout()
            writer.grab_frame()

    plt.close(fig)
    print(f"Vídeo salvo em: {output_path}")


# ---------------------------------------------------------------------------
# 5. EXECUÇÃO DE CENÁRIOS
# ---------------------------------------------------------------------------

def print_accuracy(trees: list) -> None:
    """Imprime métricas simples de erro da simulação."""
    print("\n--- Acurácia das estimativas de DAP (árvores observadas) ---")
    errors = []
    for t in trees:
        if t.num_observations > 0:
            errors.append(abs(t.dap_estimate - t.true_dap))
    if errors:
        print(f"Árvores observadas : {len(errors)}/{len(trees)}")
        print(f"Erro médio absoluto: {np.mean(errors)*100:.1f} cm")
        print(f"Erro máximo        : {np.max(errors)*100:.1f} cm")
    else:
        print("Nenhuma árvore foi observada.")


def run_scenario(scenario: str, output_path: str = None,
                 video_path: str = None,
                 verbose: bool = True) -> None:
    """Executa um cenário completo e salva gráfico + vídeo."""
    trees, bounds, description = create_forest_scenario(scenario)
    if output_path is None:
        output_path = "results.png"
    if video_path is None:
        video_path = "exploration.mp4"

    print("\n" + "#" * 72)
    print(f"Exemplo '{scenario}': {description}")
    print("#" * 72)

    tree_map, history, _ = run_mission(
        trees,
        bounds=bounds,
        scenario_name=f"{scenario} — {description}",
        verbose=verbose,
    )
    plot_results(trees, tree_map, history, output_path=output_path)
    render_exploration_video(trees, history, output_path=video_path)
    print_accuracy(trees)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simula planejamento informativo para estimar DAP."
    )
    parser.add_argument(
        "--scenario",
        choices=SCENARIOS,
        default="clusters",
        help="Cenário de floresta a simular.",
    )
    parser.add_argument(
        "--all-examples",
        action="store_true",
        help="Gera resultados para random, clusters e groves.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.all_examples:
        for scenario in SCENARIOS:
            run_scenario(
                scenario,
                output_path=f"results_{scenario}.png",
                video_path=f"exploration_{scenario}.mp4",
                verbose=True,
            )
    else:
        run_scenario(
            args.scenario,
            output_path="results.png",
            video_path="exploration.mp4",
            verbose=True,
        )
