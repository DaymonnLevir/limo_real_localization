"""
Informative RRT planner and its optional live debug visualization.

Modos de avaliação de caminho (config.RRT_SCORE_MODE):

  'hybrid' (padrão) — mistura UCB e ganho de informação:
      score(leaf) =
          w_ucb·norm(Σ UCB(nó)) +
          w_info·norm(Tr(P₀) − Tr(P_leaf)) -
          cost_weight·norm(custo)
      O termo UCB mantém o balanço entre regiões promissoras e incertas
      (μ + βσ); o termo de info gain desconta revisitas pelo fantasy GP.

  'info_gain' — ganho de informação com fantasy updates:
      score(leaf) = norm(Tr(P₀) − Tr(P_leaf)) − cost_weight·norm(custo)
      Cada nó fantasia sua medição no Kalman (ver fantasy_gp.py); a
      redução de trace é submodular, então revisitar células já cobertas
      pelo próprio caminho — ou já medidas na missão — vale ~0. Isso
      elimina o incentivo a loops e ao acampamento em regiões já vistas.

  'ucb_sum' (legado) — soma de UCB estático ao longo do caminho:
      score(leaf) = norm(Σ UCB(nó)) − cost_weight·norm(custo)
      Sem desconto por re-observação: caminhos que circulam em área de
      UCB médio acumulam mais que caminhos que atravessam zonas já
      varridas rumo à fronteira. Mantido para comparação em benchmarks.

Em todos os modos a normalização é min-max sobre os leaves da chamada
atual (escala relativa por replan) e o custo é o comprimento do caminho.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from informative_planner.fantasy_gp import FantasyGP, FantasyState
from informative_planner.visited_map import VisitedMap, VisitState


@dataclass
class RRTNode:
    """Node of the finite-horizon RRT."""

    xy: np.ndarray
    parent: Optional['RRTNode']
    ucb_accum: float = 0.0
    info_gain_accum: float = 0.0
    novelty_accum: float = 0.0
    revisit_accum: float = 0.0
    cost: float = 0.0
    depth: int = 0
    # Estado fantasiado do Kalman após as medições deste caminho
    # (None no modo 'ucb_sum'). gain_accum = Tr(P₀) − Tr(P_nó).
    fantasy: Optional[FantasyState] = field(default=None, repr=False)
    visit: Optional[VisitState] = field(default=None, repr=False)


@dataclass(frozen=True)
class RRTPathDiagnostics:
    """Summary of the selected finite-horizon RRT path."""

    first: Tuple[float, float]
    leaf: Tuple[float, float]
    path_score: float
    ucb_accum: float
    info_gain_accum: float
    info_gain_raw_accum: float
    novelty_accum: float
    revisit_accum: float
    cost: float
    depth: int
    node_count: int
    score_mode: str

    def as_dict(self) -> dict:
        return {
            'first': self.first,
            'leaf': self.leaf,
            'path_score': self.path_score,
            'ucb_accum': self.ucb_accum,
            'info_gain_accum': self.info_gain_accum,
            'info_gain_raw_accum': self.info_gain_raw_accum,
            'novelty_accum': self.novelty_accum,
            'revisit_accum': self.revisit_accum,
            'cost': self.cost,
            'depth': self.depth,
            'node_count': self.node_count,
            'score_mode': self.score_mode,
        }


def _segment_circle_collision(p1, p2, cx, cy, radius) -> bool:
    """Return whether segment ``p1 -> p2`` intersects a circle."""
    center = np.array([cx, cy], dtype=float)
    direction = p2 - p1
    squared_length = float(direction @ direction)

    if squared_length <= 1e-12:
        return bool(np.linalg.norm(p1 - center) <= radius)

    projection = float((center - p1) @ direction) / squared_length
    projection = float(np.clip(projection, 0.0, 1.0))
    closest = p1 + projection * direction
    return bool(np.linalg.norm(closest - center) <= radius)


def _is_collision_free(p1, p2, obstacles) -> bool:
    """Return whether a segment avoids every circular obstacle."""
    return not any(
        _segment_circle_collision(p1, p2, cx, cy, radius)
        for cx, cy, radius in obstacles
    )


def _nearest_node(nodes: List[RRTNode], sample: np.ndarray) -> RRTNode:
    """Return the node nearest to a sampled position."""
    points = np.asarray([node.xy for node in nodes], dtype=float)
    index = int(np.argmin(np.linalg.norm(points - sample, axis=1)))
    return nodes[index]


def _ucb_at(xy, candidates, ucb_values, radius=2.0) -> float:
    """Interpolate UCB as the local mean, falling back to the nearest cell."""
    distances = np.linalg.norm(candidates[:, :2] - xy, axis=1)
    nearby = distances < radius
    if np.any(nearby):
        return float(np.mean(ucb_values[nearby]))
    return float(ucb_values[int(np.argmin(distances))])


def _terminal_nodes(nodes: List[RRTNode], horizon: int) -> List[RRTNode]:
    """Return horizon nodes and nodes without children."""
    parent_ids = {
        id(node.parent)
        for node in nodes
        if node.parent is not None
    }
    return [
        node
        for node in nodes
        if node.depth == horizon or id(node) not in parent_ids
    ]


def _leaf_utilities(leaves: List[RRTNode], score_mode: str) -> np.ndarray:
    """Return the raw utility of each leaf according to the score mode."""
    if score_mode == 'info_gain':
        return np.asarray([node.info_gain_accum for node in leaves], dtype=float)
    return np.asarray([node.ucb_accum for node in leaves], dtype=float)


def _normalize(values: np.ndarray) -> np.ndarray:
    """Min-max normalize a vector, returning zeros when all values match."""
    values = np.asarray(values, dtype=float)
    value_range = float(np.ptp(values))
    if value_range <= 1e-12:
        return np.zeros_like(values)
    return (values - np.min(values)) / value_range


def _path_utilities(
    leaves: List[RRTNode],
    score_mode: str,
    ucb_weight: float,
    info_gain_weight: float,
    visit_novelty_weight: float,
    info_gain_norm: str = 'minmax',
    info_gain_ref: Optional[float] = None,
) -> np.ndarray:
    """Blend normalized positive utility terms per leaf.

    info_gain_norm='minmax' (legado) renormaliza o ganho por replan, mantendo
    seu peso relativo constante ao longo da missão. 'absolute' divide o ganho
    acumulado por uma referência FIXA do prior (info_gain_ref): conforme a
    incerteza global colapsa, o termo decai em escala natural e o UCB passa a
    dominar a decisão progressivamente.
    """
    terms = []

    if score_mode == 'hybrid':
        terms.append((
            float(ucb_weight),
            _normalize(np.asarray(
                [node.ucb_accum for node in leaves],
                dtype=float,
            )),
        ))
        gains = np.asarray(
            [node.info_gain_accum for node in leaves],
            dtype=float,
        )
        if (
            info_gain_norm == 'absolute'
            and info_gain_ref is not None
            and info_gain_ref > 0.0
        ):
            gain_term = np.clip(gains / float(info_gain_ref), 0.0, 1.0)
        else:
            gain_term = _normalize(gains)
        terms.append((float(info_gain_weight), gain_term))
    else:
        terms.append((
            1.0,
            _normalize(_leaf_utilities(leaves, score_mode)),
        ))

    if visit_novelty_weight > 0.0:
        terms.append((
            float(visit_novelty_weight),
            _normalize(np.asarray(
                [node.novelty_accum for node in leaves],
                dtype=float,
            )),
        ))

    weight_sum = sum(weight for weight, _ in terms if weight > 0.0)
    if weight_sum <= 1e-12:
        return np.zeros(len(leaves), dtype=float)

    utilities = np.zeros(len(leaves), dtype=float)
    for weight, values in terms:
        if weight > 0.0:
            utilities += weight * values
    return utilities / weight_sum


def _best_leaf_with_score(
    nodes: List[RRTNode],
    horizon: int,
    cost_weight: float,
    score_mode: str = 'ucb_sum',
    hybrid_ucb_weight: float = 0.6,
    hybrid_info_gain_weight: float = 0.4,
    visit_novelty_weight: float = 0.0,
    visit_penalty_weight: float = 0.0,
    info_gain_norm: str = 'minmax',
    info_gain_ref: Optional[float] = None,
) -> Tuple[RRTNode, float]:
    """Select the terminal node and return its normalized path score."""
    leaves = _terminal_nodes(nodes, horizon)
    leaf_costs = np.asarray([node.cost for node in leaves], dtype=float)

    normalized_utils = _path_utilities(
        leaves,
        score_mode,
        hybrid_ucb_weight,
        hybrid_info_gain_weight,
        visit_novelty_weight,
        info_gain_norm,
        info_gain_ref,
    )
    normalized_costs = _normalize(leaf_costs)
    normalized_revisits = _normalize(np.asarray(
        [node.revisit_accum for node in leaves],
        dtype=float,
    ))

    scores = (
        normalized_utils
        - cost_weight * normalized_costs
        - visit_penalty_weight * normalized_revisits
    )
    best_index = int(np.argmax(scores))
    return leaves[best_index], float(scores[best_index])


def _best_leaf(
    nodes: List[RRTNode],
    horizon: int,
    cost_weight: float,
    score_mode: str = 'ucb_sum',
    hybrid_ucb_weight: float = 0.6,
    hybrid_info_gain_weight: float = 0.4,
    visit_novelty_weight: float = 0.0,
    visit_penalty_weight: float = 0.0,
) -> RRTNode:
    """Select the terminal node using normalized utility and path cost."""
    return _best_leaf_with_score(
        nodes,
        horizon,
        cost_weight,
        score_mode,
        hybrid_ucb_weight,
        hybrid_info_gain_weight,
        visit_novelty_weight,
        visit_penalty_weight,
    )[0]


def _path_from_leaf(leaf: RRTNode) -> List[np.ndarray]:
    """Recover an ordered path, excluding the root node."""
    path = []
    node = leaf
    while node.parent is not None:
        path.append(node.xy.copy())
        node = node.parent
    path.reverse()
    return path


class _RRTDebugPlot:
    """Non-blocking Matplotlib view of the RRT expansion."""

    def __init__(
        self,
        candidates,
        ucb_values,
        obstacles,
        map_bounds,
        horizon,
        cost_weight,
        pause_sec,
        score_mode='ucb_sum',
        hybrid_ucb_weight=0.6,
        hybrid_info_gain_weight=0.4,
        visit_novelty_weight=0.0,
        visit_penalty_weight=0.0,
    ):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.candidates = candidates
        self.ucb_values = ucb_values
        self.obstacles = obstacles
        self.map_bounds = map_bounds
        self.horizon = horizon
        self.cost_weight = cost_weight
        self.pause_sec = max(float(pause_sec), 0.0)
        self.score_mode = score_mode
        self.hybrid_ucb_weight = hybrid_ucb_weight
        self.hybrid_info_gain_weight = hybrid_info_gain_weight
        self.visit_novelty_weight = visit_novelty_weight
        self.visit_penalty_weight = visit_penalty_weight

        plt.ion()
        self.figure = plt.figure('Informative RRT debug', figsize=(8, 7))
        self.figure.clear()
        self.axes = self.figure.add_subplot(111)
        self.colorbar = None

    def update(self, nodes, iteration, sample, status, final=False):
        """Redraw the complete search state."""
        from matplotlib.patches import Circle

        axes = self.axes
        axes.clear()
        scatter = axes.scatter(
            self.candidates[:, 0],
            self.candidates[:, 1],
            c=self.ucb_values,
            cmap='viridis',
            marker='s',
            s=45,
            alpha=0.65,
            label='UCB grid',
            zorder=1,
        )
        if self.colorbar is None:
            self.colorbar = self.figure.colorbar(scatter, ax=axes)
            self.colorbar.set_label('UCB')
        else:
            self.colorbar.update_normal(scatter)

        for index, (cx, cy, radius) in enumerate(self.obstacles):
            axes.add_patch(Circle(
                (cx, cy),
                radius,
                facecolor='tab:red',
                edgecolor='darkred',
                alpha=0.35,
                label='Tree obstacle' if index == 0 else None,
                zorder=2,
            ))

        for index, node in enumerate(nodes):
            if node.parent is None:
                continue
            axes.plot(
                [node.parent.xy[0], node.xy[0]],
                [node.parent.xy[1], node.xy[1]],
                color='tab:blue',
                linewidth=0.8,
                alpha=0.55,
                label='RRT edges' if index == 1 else None,
                zorder=3,
            )

        node_points = np.asarray([node.xy for node in nodes], dtype=float)
        axes.scatter(
            node_points[:, 0],
            node_points[:, 1],
            color='tab:blue',
            s=12,
            zorder=4,
        )
        axes.scatter(
            nodes[0].xy[0],
            nodes[0].xy[1],
            color='lime',
            edgecolor='black',
            marker='*',
            s=180,
            label='Start',
            zorder=7,
        )

        best_path = _path_from_leaf(
            _best_leaf(
                nodes,
                self.horizon,
                self.cost_weight,
                self.score_mode,
                self.hybrid_ucb_weight,
                self.hybrid_info_gain_weight,
                self.visit_novelty_weight,
                self.visit_penalty_weight,
            )
        )
        if best_path:
            path_points = np.vstack([nodes[0].xy, *best_path])
            axes.plot(
                path_points[:, 0],
                path_points[:, 1],
                color='magenta',
                linewidth=3.0,
                label='Current best path',
                zorder=6,
            )

        if sample is not None:
            axes.scatter(
                sample[0],
                sample[1],
                color='orange',
                marker='x',
                s=70,
                label='Current sample',
                zorder=8,
            )

        x_min, x_max, y_min, y_max = self.map_bounds
        axes.set_xlim(x_min, x_max)
        axes.set_ylim(y_min, y_max)
        axes.set_aspect('equal', adjustable='box')
        axes.set_xlabel('X [m]')
        axes.set_ylabel('Y [m]')
        phase = 'final' if final else f'iteration {iteration}'
        axes.set_title(
            f'Informative RRT - {phase} | nodes={len(nodes)} | {status}'
        )
        axes.legend(loc='upper right', fontsize=8)
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        if self.pause_sec > 0.0:
            self.plt.pause(self.pause_sec)


def rrt_informative(
    start_xy: np.ndarray,
    candidates: np.ndarray,
    ucb_values: np.ndarray,
    obstacles: list,
    map_bounds: Tuple[float, float, float, float],
    n_iter: int = 300,
    step_size: float = 5.0,
    horizon: int = 5,
    goal_bias: float = 0.15,
    cost_weight: float = 0.3,
    debug_plot: bool = False,
    debug_plot_every: int = 1,
    debug_pause_sec: float = 0.01,
    fantasy_gp: Optional[FantasyGP] = None,
    visited_map: Optional[VisitedMap] = None,
    score_mode: str = 'ucb_sum',
    hybrid_ucb_weight: float = 0.6,
    hybrid_info_gain_weight: float = 0.4,
    visit_novelty_weight: float = 0.0,
    visit_penalty_weight: float = 0.0,
    future_discount: float = 1.0,
    return_diagnostics: bool = False,
    info_gain_norm: str = 'minmax',
    info_gain_ref: Optional[float] = None,
):
    """
    Return the best finite-horizon path and its first-waypoint UCB.

    If return_diagnostics=True, also return a dict with the selected path's
    first waypoint, leaf, normalized path score, accumulated UCB,
    accumulated information gain, cost, depth, and node count.

    Parâmetros adicionais
    ---------------------
    fantasy_gp : FantasyGP, opcional
        Snapshot do posterior atual do ExplorationGP. Obrigatório quando
        score_mode usa ganho de informação: cada nó da árvore fantasia sua
        medição (downdate rank-1 do Kalman) e o leaf considera a redução
        de trace acumulada do caminho. Ver fantasy_gp.py.
    score_mode : {'ucb_sum', 'info_gain', 'hybrid'}
        Critério de avaliação dos leaves (ver docstring do módulo).
    hybrid_ucb_weight : float
        Peso do termo UCB no modo 'hybrid'.
    hybrid_info_gain_weight : float
        Peso do termo ganho de informação no modo 'hybrid'.

    O UCB continua sendo usado para o goal_bias da amostragem (puxar a
    expansão para a região mais promissora) e para o score retornado do
    primeiro waypoint. No modo 'hybrid', ele também participa diretamente
    da seleção do melhor caminho.
    """
    candidates = np.asarray(candidates, dtype=float)
    ucb_values = np.asarray(ucb_values, dtype=float).reshape(-1)
    start_xy = np.asarray(start_xy[:2], dtype=float)

    if score_mode not in ('ucb_sum', 'info_gain', 'hybrid'):
        raise ValueError(f'score_mode inválido: {score_mode!r}')
    if info_gain_norm not in ('minmax', 'absolute'):
        raise ValueError(f'info_gain_norm inválido: {info_gain_norm!r}')
    if info_gain_norm == 'absolute' and (
        info_gain_ref is None or float(info_gain_ref) <= 0.0
    ):
        raise ValueError("info_gain_norm='absolute' requer info_gain_ref > 0")
    if score_mode in ('info_gain', 'hybrid') and fantasy_gp is None:
        raise ValueError(f"score_mode={score_mode!r} requer fantasy_gp")
    if hybrid_ucb_weight < 0.0 or hybrid_info_gain_weight < 0.0:
        raise ValueError('hybrid weights must be non-negative')
    if visit_novelty_weight < 0.0 or visit_penalty_weight < 0.0:
        raise ValueError('visit weights must be non-negative')
    if not 0.0 < future_discount <= 1.0:
        raise ValueError('future_discount must be in (0, 1]')
    if (
        score_mode == 'hybrid'
        and hybrid_ucb_weight + hybrid_info_gain_weight <= 1e-12
    ):
        raise ValueError('hybrid mode requires at least one positive weight')

    if candidates.ndim != 2 or candidates.shape[0] == 0:
        raise ValueError('candidates must be a non-empty 2-D array')
    if candidates.shape[1] < 2:
        raise ValueError('candidates must contain x and y coordinates')
    if len(ucb_values) != len(candidates):
        raise ValueError('ucb_values and candidates must have equal length')
    if n_iter <= 0 or horizon <= 0 or step_size <= 0.0:
        raise ValueError('n_iter, horizon, and step_size must be positive')
    if not 0.0 <= goal_bias <= 1.0:
        raise ValueError('goal_bias must be between 0 and 1')

    x_min, x_max, y_min, y_max = map_bounds
    lower_bounds = np.array([x_min, y_min], dtype=float)
    upper_bounds = np.array([x_max, y_max], dtype=float)
    best_candidate = candidates[int(np.argmax(ucb_values)), :2]

    fantasy_root = fantasy_gp.root() if fantasy_gp is not None else None
    visit_root = visited_map.root() if visited_map is not None else None
    root = RRTNode(
        xy=start_xy.copy(),
        parent=None,
        fantasy=fantasy_root,
        visit=visit_root,
    )
    nodes = [root]
    plotter = None
    if debug_plot:
        plotter = _RRTDebugPlot(
            candidates,
            ucb_values,
            obstacles,
            map_bounds,
            horizon,
            cost_weight,
            debug_pause_sec,
            score_mode,
            hybrid_ucb_weight,
            hybrid_info_gain_weight,
            visit_novelty_weight,
            visit_penalty_weight,
        )
    plot_every = max(int(debug_plot_every), 1)

    for iteration in range(1, n_iter + 1):
        if np.random.rand() < goal_bias:
            sample = best_candidate + np.random.randn(2) * step_size * 0.5
        else:
            sample = np.array([
                np.random.uniform(x_min, x_max),
                np.random.uniform(y_min, y_max),
            ])
        sample = np.clip(sample, lower_bounds, upper_bounds)

        status = 'accepted'
        expandable_nodes = [node for node in nodes if node.depth < horizon]
        if not expandable_nodes:
            status = 'horizon reached'
        else:
            nearest = _nearest_node(expandable_nodes, sample)
            direction = sample - nearest.xy
            distance = float(np.linalg.norm(direction))
            if distance < 1e-6:
                status = 'rejected: duplicate'
            else:
                new_xy = nearest.xy + (
                    direction / distance
                ) * min(step_size, distance)
                if not _is_collision_free(nearest.xy, new_xy, obstacles):
                    status = 'rejected: collision'
                else:
                    edge_cost = float(np.linalg.norm(new_xy - nearest.xy))
                    node_ucb = _ucb_at(new_xy, candidates, ucb_values)
                    node_fantasy = (
                        nearest.fantasy.observe_at(new_xy)
                        if nearest.fantasy is not None else None
                    )
                    node_visit = (
                        nearest.visit.observe_at(new_xy)
                        if nearest.visit is not None else None
                    )
                    discount = float(future_discount) ** nearest.depth
                    node_info_gain = (
                        node_fantasy.gain
                        if node_fantasy is not None else 0.0
                    )
                    node_novelty = (
                        node_visit.novelty
                        if node_visit is not None else 0.0
                    )
                    node_revisit = (
                        node_visit.revisit
                        if node_visit is not None else 0.0
                    )
                    nodes.append(RRTNode(
                        xy=new_xy,
                        parent=nearest,
                        ucb_accum=(
                            nearest.ucb_accum + discount * node_ucb
                        ),
                        info_gain_accum=(
                            nearest.info_gain_accum
                            + discount * node_info_gain
                        ),
                        novelty_accum=(
                            nearest.novelty_accum
                            + discount * node_novelty
                        ),
                        revisit_accum=(
                            nearest.revisit_accum
                            + discount * node_revisit
                        ),
                        cost=nearest.cost + edge_cost,
                        depth=nearest.depth + 1,
                        fantasy=node_fantasy,
                        visit=node_visit,
                    ))

        if plotter is not None and (
            iteration % plot_every == 0 or iteration == n_iter
        ):
            plotter.update(nodes, iteration, sample, status)

    best_leaf, path_score = _best_leaf_with_score(
        nodes,
        horizon,
        cost_weight,
        score_mode,
        hybrid_ucb_weight,
        hybrid_info_gain_weight,
        visit_novelty_weight,
        visit_penalty_weight,
        info_gain_norm,
        info_gain_ref,
    )
    path = _path_from_leaf(best_leaf)
    if plotter is not None:
        plotter.update(nodes, n_iter, None, 'selected path', final=True)
    if not path:
        if return_diagnostics:
            return [], float('-inf'), None
        return [], float('-inf')

    first_score = _ucb_at(path[0], candidates, ucb_values)
    if not return_diagnostics:
        return path, first_score

    info_gain_accum = (
        best_leaf.info_gain_accum
    )
    info_gain_raw_accum = (
        best_leaf.fantasy.gain_accum
        if best_leaf.fantasy is not None else best_leaf.info_gain_accum
    )
    diagnostics = RRTPathDiagnostics(
        first=(float(path[0][0]), float(path[0][1])),
        leaf=(float(best_leaf.xy[0]), float(best_leaf.xy[1])),
        path_score=float(path_score),
        ucb_accum=float(best_leaf.ucb_accum),
        info_gain_accum=float(info_gain_accum),
        info_gain_raw_accum=float(info_gain_raw_accum),
        novelty_accum=float(best_leaf.novelty_accum),
        revisit_accum=float(best_leaf.revisit_accum),
        cost=float(best_leaf.cost),
        depth=int(best_leaf.depth),
        node_count=len(nodes),
        score_mode=score_mode,
    )
    return path, first_score, diagnostics.as_dict()
