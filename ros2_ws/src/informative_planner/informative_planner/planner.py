"""
planner.py
----------
Planejador baseado em UCB do GP de exploração + bônus angular analítico.
"""

from typing import List, Optional, Tuple

import numpy as np

from informative_planner.rrt import rrt_informative
from informative_planner import config
from informative_planner.exploration_gp import ExplorationGP
from informative_planner.fantasy_gp import FantasyGP
from informative_planner.visited_map import VisitedMap


def generate_waypoint_candidates(
    forest_bounds: Tuple[float, float, float, float],
    resolution: float = None,
    height: float = None,
) -> np.ndarray:
    """Gera uma grade regular de waypoints candidatos."""
    if resolution is None:
        resolution = config.WAYPOINT_GRID_RESOLUTION
    if height is None:
        height = config.FLIGHT_HEIGHT

    if resolution <= 0.0:
        raise ValueError('resolution must be positive')

    x_min, x_max, y_min, y_max = forest_bounds

    # Grade que cobre EXATAMENTE os bounds do mapa, incluindo a borda superior.
    # np.arange(x_min, x_max, res) parava antes de x_max quando o vão não era
    # múltiplo de `resolution` (ex.: [-16, 16] com res=3 ia só até 14), fazendo
    # o grid do planner ficar menor que o mapa real. linspace com os dois
    # extremos inclusos corrige isso; a resolução efetiva se ajusta ao vão.
    def _axis(lo, hi):
        span = float(hi) - float(lo)
        n_intervals = max(1, int(round(span / resolution)))
        return np.linspace(float(lo), float(hi), n_intervals + 1)

    xs = _axis(x_min, x_max)
    ys = _axis(y_min, y_max)
    candidates = []
    for x in xs:
        for y in ys:
            candidates.append([x, y, height])
    return np.array(candidates)


# ---------------------------------------------------------------------------
# Referência fixa de ganho para RRT_INFO_GAIN_NORMALIZATION='absolute'
# ---------------------------------------------------------------------------

_PRIOR_GAIN_REF_CACHE: dict = {}


def _prior_path_gain_ref(exploration_gp: ExplorationGP) -> float:
    """Ganho de trace de um caminho cheio sobre o PRIOR virgem do GP.

    Referência fixa (não muda com a missão) usada para pôr o termo de info
    gain do score híbrido em escala ~[0, 1]: Delta0 * sum(gamma^d), onde
    Delta0 é o ganho de UMA observação no prior. Assim o termo decai em
    escala natural conforme a incerteza colapsa, e o UCB passa a dominar.
    """
    key = (
        exploration_gp.M,
        float(exploration_gp.ell),
        float(exploration_gp.sf),
        float(exploration_gp.noise_var),
        exploration_gp.observation_model.mode,
        float(exploration_gp.observation_model.footprint_radius),
        float(exploration_gp.observation_model.footprint_sigma),
        int(config.RRT_HORIZON),
        float(getattr(config, 'RRT_FUTURE_DISCOUNT', 1.0)),
    )
    cached = _PRIOR_GAIN_REF_CACHE.get(key)
    if cached is not None:
        return cached

    grid = exploration_gp.X_grid
    p0 = exploration_gp._matern32(grid, grid) + (
        exploration_gp.noise_var * np.eye(exploration_gp.M)
    )
    center = grid[:, :2].mean(axis=0)
    h = exploration_gp.observation_model.vector_at(center)
    p_h = p0[:, h.indices] @ h.weights
    s = float(h.weights @ p_h[h.indices]) + exploration_gp.noise_var
    delta0 = float(p_h @ p_h) / s if s > 0.0 else 1.0

    gamma = float(getattr(config, 'RRT_FUTURE_DISCOUNT', 1.0))
    horizon = max(int(config.RRT_HORIZON), 1)
    discount_sum = sum(gamma ** d for d in range(horizon))

    ref = max(delta0 * discount_sum, 1e-9)
    _PRIOR_GAIN_REF_CACHE[key] = ref
    return ref


# ---------------------------------------------------------------------------
# Planejamento baseado no GP de exploração (UCB) + bônus angular analítico
# ---------------------------------------------------------------------------

def compute_visit_utility(visible_indices: List[int]) -> float:
    """
    Utilidade observada pelo GP ao visitar uma posição: apenas contagem de
    árvores visíveis.

    O GP mapeia exclusivamente esta propriedade **estática** do espaço —
    quantas árvores existem em cada região. Como essa contagem não muda ao
    longo da missão (as árvores não se movem), o GP aprende uma função
    consistente e interpola corretamente para regiões não visitadas.

    O bônus por novos ângulos é intencionalmenteexcluído daqui porque ele é
    **estado-dependente**: a mesma posição tem bônus alto no início (muitos
    ângulos novos) e baixo no final (ângulos já cobertos). Incluí-lo
    contaminaria o GP com variação temporal, não espacial.
    """
    return float(len(visible_indices))


def compute_angular_bonus(
    cand_xy: np.ndarray,
    trees: list,
    alpha: float = 1.5,
    max_range: float = None,
) -> float:
    """
    Bônus analítico por novos setores angulares em um candidato a waypoint.

    Para árvores já **descobertas** (num_observations > 0), sabemos posição
    e ângulos já registrados — o $N_{\\text{new}}$ pode ser calculado
    exatamente sem nenhuma aproximação por GP. Adicioná-lo diretamente ao
    critério de seleção garante que a informação angular esteja sempre
    atualizada, ao contrário de deixá-la ser suavizada pelo posterior do GP.

    Parâmetros
    ----------
    cand_xy : np.ndarray, shape (2,)
        Posição candidata.
    trees : list
        Lista completa de árvores.
    alpha : float
        Peso do bônus angular (mesmo alpha da função de utilidade original).
    max_range : float, opcional
        Alcance máximo do LiDAR. Usa config.LIDAR_MAX_RANGE se None.

    Retorna
    -------
    float
        alpha * (número de árvores conhecidas que ganhariam novo setor angular).
    """
    if max_range is None:
        max_range = config.LIDAR_MAX_RANGE

    n_new = 0
    for tree in trees:
        if tree.num_observations == 0:
            continue  # ainda não descoberta: posição desconhecida
        dist = np.linalg.norm(np.array([tree.x, tree.y]) - cand_xy)
        if dist > max_range:
            continue  # fora do alcance
        angle = tree.observation_angle_from(cand_xy)
        if tree.angle_sector_count(extra_angle=angle) > tree.angle_sector_count():
            n_new += 1

    return alpha * float(n_new)



def _greedy_selection(
    exploration_gp: ExplorationGP,
    candidates: np.ndarray,
    drone_xy: np.ndarray,
    trees: list = None,
    min_dist: float = 1.5,
    max_step: float = None,
    alpha: float = 1.5,
    beta: float = 0.5,
) -> Tuple[Optional[np.ndarray], float]:
    if max_step is None:
        max_step = config.MAX_WAYPOINT_STEP

    candidates = np.asarray(candidates, dtype=float)
    drone_xy = np.asarray(drone_xy[:2], dtype=float)

    if candidates.ndim != 2 or candidates.shape[0] == 0:
        return None, float('-inf')
    if candidates.shape[1] < 2:
        return None, float('-inf')
    if not np.all(np.isfinite(candidates[:, :2])):
        return None, float('-inf')
    if not np.all(np.isfinite(drone_xy)):
        return None, float('-inf')

    cand_xy = candidates[:, :2]
    scores = exploration_gp.ucb_grid().copy()
    scores = np.asarray(scores, dtype=float)

    if scores.shape[0] != candidates.shape[0]:
        return None, float('-inf')

    # Bônus analítico por ângulos novos de árvores já conhecidas
    # if trees is not None:
    #     for i, cand in enumerate(cand_xy):
    #         scores[i] += compute_angular_bonus(cand, trees, alpha=alpha)

    dists = np.linalg.norm(cand_xy - drone_xy, axis=1)
    feasible = np.isfinite(scores) & np.isfinite(dists)

    if min_dist is not None and min_dist > 0.0:
        feasible &= dists >= min_dist

    if max_step is not None:
        feasible &= dists <= max_step

    if not np.any(feasible):
        return None, float('-inf')

    feasible_indices = np.flatnonzero(feasible)
    best_idx = int(feasible_indices[np.argmax(scores[feasible])])

    return candidates[best_idx], float(scores[best_idx])



def find_next_waypoint_ucb(
    exploration_gp: ExplorationGP,
    candidates: np.ndarray,
    drone_xy: np.ndarray,
    trees: list = None,
    min_dist: float = 1.5,
    max_step: float = None,
    alpha: float = 1.5,
    beta: float = 0.5,
    map_bounds: Tuple[float, float, float, float] = None,
    visited_map: VisitedMap = None,
    return_plan_info: bool = False,
) -> tuple:
    planejamento = config.PLANEJAMENTO

    if planejamento == 'greedy_selection':
        waypoint, score = _greedy_selection(
            exploration_gp,
            candidates,
            drone_xy,
            trees,
            min_dist,
            max_step,
            alpha,
            beta,
        )
        if not return_plan_info:
            return waypoint, score
        plan_info = None
        if waypoint is not None:
            plan_info = {
                'planner': 'greedy_selection',
                'first': (
                    float(waypoint[0]),
                    float(waypoint[1]),
                ),
                'leaf': (
                    float(waypoint[0]),
                    float(waypoint[1]),
                ),
                'path_score': float(score),
                'ucb_accum': float(score),
                'info_gain_accum': 0.0,
                'cost': 0.0,
                'depth': 1,
                'node_count': 1,
                'score_mode': 'greedy_selection',
            }
        return waypoint, score, plan_info
    elif planejamento == 'rrt_informative':
        # RRT - Planejamento NÃO miópe. Considera os 5 próximos passos
        obstacles = [
            (float(tree.x), float(tree.y), max(float(tree.radius), 0.0))
            for tree in (trees or [])
        ]
        score_mode = getattr(config, 'RRT_SCORE_MODE', 'ucb_sum')
        # Snapshot do posterior atual para os fantasy updates (uma cópia
        # de P por replan; O(M²) ≈ trivial para a grade de waypoints).
        fantasy_gp = (
            FantasyGP(exploration_gp)
            if score_mode in ('info_gain', 'hybrid') else None
        )
        info_gain_norm = getattr(
            config, 'RRT_INFO_GAIN_NORMALIZATION', 'minmax'
        )
        info_gain_ref = (
            _prior_path_gain_ref(exploration_gp)
            if info_gain_norm == 'absolute' and score_mode == 'hybrid'
            else None
        )
        if info_gain_norm == 'absolute' and score_mode != 'hybrid':
            info_gain_norm = 'minmax'
        rrt_result = rrt_informative(
            start_xy=drone_xy,
            candidates=candidates,
            ucb_values=exploration_gp.ucb_grid(),
            obstacles=obstacles,
            map_bounds=(
                config.MAP_BOUNDS if map_bounds is None else map_bounds
            ),
            n_iter=config.RRT_N_ITER,
            step_size=min(
                config.RRT_STEP_SIZE,
                max_step if max_step is not None else config.RRT_STEP_SIZE,
            ),
            horizon=config.RRT_HORIZON,
            goal_bias=config.RRT_GOAL_BIAS,
            cost_weight=config.RRT_COST_WEIGHT,
            debug_plot=config.RRT_DEBUG_PLOT,
            debug_plot_every=config.RRT_DEBUG_PLOT_EVERY,
            debug_pause_sec=config.RRT_DEBUG_PLOT_PAUSE_SEC,
            fantasy_gp=fantasy_gp,
            visited_map=visited_map,
            score_mode=score_mode,
            hybrid_ucb_weight=getattr(
                config,
                'RRT_HYBRID_UCB_WEIGHT',
                0.6,
            ),
            hybrid_info_gain_weight=getattr(
                config,
                'RRT_HYBRID_INFO_GAIN_WEIGHT',
                0.4,
            ),
            visit_novelty_weight=getattr(
                config,
                'RRT_VISIT_NOVELTY_WEIGHT',
                0.0,
            ),
            visit_penalty_weight=getattr(
                config,
                'RRT_VISIT_PENALTY_WEIGHT',
                0.0,
            ),
            future_discount=getattr(
                config,
                'RRT_FUTURE_DISCOUNT',
                1.0,
            ),
            return_diagnostics=return_plan_info,
            info_gain_norm=info_gain_norm,
            info_gain_ref=info_gain_ref,
        )
        if return_plan_info:
            best_path, score, plan_info = rrt_result
        else:
            best_path, score = rrt_result
            plan_info = None
        if not best_path:
            if return_plan_info:
                return None, float('-inf'), plan_info
            return None, float('-inf')

        next_xy = best_path[0]
        next_waypoint = np.array([
            next_xy[0],
            next_xy[1],
            config.FLIGHT_HEIGHT,
        ])
        if return_plan_info:
            if plan_info is not None:
                plan_info = dict(plan_info)
                plan_info['planner'] = 'rrt_informative'
                plan_info['first_waypoint_score'] = float(score)
                plan_info['path_length'] = len(best_path)
                plan_info['path'] = [
                    (float(point[0]), float(point[1]))
                    for point in best_path
                ]
            return next_waypoint, score, plan_info
        return next_waypoint, score

    raise ValueError(f'Estratégia de planejamento inválida: {planejamento}')
