from __future__ import annotations

import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle, FancyArrowPatch

from informative_planner import config
from informative_planner.exploration_gp import ExplorationGP
from informative_planner.fantasy_gp import FantasyGP
from informative_planner.planner import generate_waypoint_candidates
from informative_planner.rrt import (
    RRTNode,
    _is_collision_free,
    _nearest_node,
    _ucb_at,
    _terminal_nodes,
    _path_from_leaf,
)
from informative_planner.visited_map import VisitedMap

try:
    from pptx import Presentation
    from pptx.util import Inches
    PPTX_AVAILABLE = True
except Exception:
    PPTX_AVAILABLE = False

DEFAULT_RUN = '/root/mrs_ws/src/octomap_gazebo_forest/results/2026-07-16/informative/forest_clustered_10min/13-34-37/forest_clustered_10min_informative_rep01'
DEFAULT_GT = '/root/mrs_ws/src/octomap_gazebo_forest/ground_truth/forest_clustered_10min/forest_clustered_10min_ground_truth.csv'

# Estas globais são definidas em configure() a partir de --run.
RUN = Path(DEFAULT_RUN)
GT_PATH = Path(DEFAULT_GT)
OUT = RUN / 'analysis' / 'rrt_presentation_deck'
SLIDES = OUT / 'slides_png'
RUN_LABEL = ''
RUN_SUMMARY = {}


def configure(run_dir):
    """Aponta o gerador para um run (rep) específico e cria as pastas de saída."""
    global RUN, GT_PATH, OUT, SLIDES, RUN_LABEL, RUN_SUMMARY, BOUNDS
    RUN = Path(run_dir).resolve()
    RUN_SUMMARY = {}
    summ = RUN / 'run_summary.json'
    if summ.exists():
        try:
            RUN_SUMMARY = json.loads(summ.read_text())
        except Exception:
            RUN_SUMMARY = {}
    gt = (RUN_SUMMARY.get('metadata', {}) or {}).get('ground_truth_csv_path')
    GT_PATH = Path(gt) if gt and Path(gt).exists() else Path(DEFAULT_GT)
    OUT = RUN / 'analysis' / 'rrt_presentation_deck'
    SLIDES = OUT / 'slides_png'
    OUT.mkdir(parents=True, exist_ok=True)
    SLIDES.mkdir(parents=True, exist_ok=True)
    # rótulo tipo "13-59-32 · rep01"
    parts = RUN.parts
    stamp = next((p for p in parts if re.fullmatch(r'\d{2}-\d{2}-\d{2}', p)), '')
    rep = next((p for p in parts if 'rep' in p), RUN.name)
    RUN_LABEL = ' · '.join(x for x in (RUN.parts[-3] if len(RUN.parts) >= 3 else '', stamp, rep) if x)
    mb = (RUN_SUMMARY.get('metadata', {}) or {}).get('map_bounds')
    if mb and len(mb) == 4:
        BOUNDS = tuple(float(v) for v in mb)
    return RUN


BOUNDS = (-16.0, 16.0, -16.0, 16.0)
W_UCB = float(config.RRT_HYBRID_UCB_WEIGHT)
W_INFO = float(config.RRT_HYBRID_INFO_GAIN_WEIGHT)
W_NOV = float(config.RRT_VISIT_NOVELTY_WEIGHT)
W_COST = float(config.RRT_COST_WEIGHT)
W_REV = float(config.RRT_VISIT_PENALTY_WEIGHT)
WEIGHT_SUM = W_UCB + W_INFO + W_NOV
COMMIT = int(getattr(config, 'RRT_COMMITTED_WAYPOINTS', 3))
STEP = float(config.RRT_STEP_SIZE)

# ── Estética: mesmo tema escuro do gp_propagation.mp4 ─────────────────────
BG = '#101418'
PANEL = '#161b21'
INK = '#e8ecef'
MUT = '#9aa4ad'
GRID = '#2a3138'
GREEN = '#5ad19a'
YELLOW = '#ffd34d'
DRONE = '#ffffff'
RED = '#ff6b6b'
CMAP_MEAN = 'viridis'
CMAP_STD = 'magma'
CMAP_UCB = 'viridis'
CMAP_REVISIT = 'inferno'
BAR_COLORS = {
    'utility': '#2dd4bf', 'UCB': '#38bdf8', 'I': '#4ade80',
    'Nov': '#a3e635', '-C': '#fb923c', '-Rev': '#f87171',
}
GROUP_COLORS = {'top': '#22c55e', 'mid': '#38bdf8', 'bot': '#f87171'}

# Os momentos-chave são escolhidos automaticamente por select_moments():
# ~5 replans espalhados no tempo + o replan em que o termo −Rev decide.
rng_seed_base = 4100


@dataclass
class PlanRecord:
    index: int
    line: int
    elapsed: float
    kind: str
    obs_count: int
    map_count: int
    logged_first: tuple[float, float]
    logged_leaf: tuple[float, float]
    logged_score: float
    logged_novelty: float
    logged_revisit: float
    logged_cost: float
    logged_path: tuple = ()
    logged_depth: int = 0


def read_gt():
    with GT_PATH.open() as f:
        return pd.DataFrame([{'id': int(r['id']), 'x': float(r['x']), 'y': float(r['y'])} for r in csv.DictReader(f)])


def parse_plans():
    lines = (RUN / 'gaussian_feeder.log').read_text(errors='replace').splitlines()
    obs = 0; mapc = 0; start_ts = None; records = []
    plan_re = re.compile(
        r'\[(\d+\.\d+)\].*RRT path (?P<kind>fresh|precomputed): .*?'
        r'first=\[(?P<fx>-?\d+\.\d+), (?P<fy>-?\d+\.\d+)\] '
        r'leaf=\[(?P<lx>-?\d+\.\d+), (?P<ly>-?\d+\.\d+)\].*?'
        r'path_score=(?P<score>-?\d+\.\d+).*?'
        r'novelty_accum=(?P<nov>-?\d+\.\d+) '
        r'revisit_accum=(?P<rev>-?\d+\.\d+).*?cost=(?P<cost>-?\d+\.\d+)'
    )
    for line_no, line in enumerate(lines, start=1):
        ts_match = re.search(r'\[(\d+\.\d+)\]', line)
        if ts_match and start_ts is None:
            start_ts = float(ts_match.group(1))
        m = re.search(r'GP atualizado: .* obs=(\d+)', line)
        if m:
            obs = int(m.group(1))
        m = re.search(r'Mapa atualizado: (\d+) árvores', line)
        if m:
            mapc = int(m.group(1))
        m = plan_re.search(line)
        if m:
            t = float(m.group(1))
            pts = ()
            depth = 0
            pm = re.search(r'path=\[(.*)\] depth=(\d+)', line)
            if pm:
                pts = tuple((float(a), float(b)) for a, b in re.findall(r'\[(-?\d+\.\d+), (-?\d+\.\d+)\]', pm.group(1)))
                depth = int(pm.group(2))
            records.append(PlanRecord(
                index=len(records), line=line_no, elapsed=t - (start_ts or t), kind=m.group('kind'),
                obs_count=obs, map_count=mapc,
                logged_first=(float(m.group('fx')), float(m.group('fy'))),
                logged_leaf=(float(m.group('lx')), float(m.group('ly'))),
                logged_score=float(m.group('score')), logged_novelty=float(m.group('nov')),
                logged_revisit=float(m.group('rev')), logged_cost=float(m.group('cost')),
                logged_path=pts, logged_depth=depth,
            ))
    return records


def load_snapshots():
    with (RUN / 'tree_maps/tree_mapper_snapshots.jsonl').open() as f:
        return [json.loads(line) for line in f if line.strip()]


def snapshot_at(snaps, elapsed):
    if not snaps:
        return {'trees': [], 'tree_count': 0, 'time_sec': 0.0}
    idx = int(np.argmin([abs(float(s['time_sec']) - elapsed) for s in snaps]))
    return snaps[idx]


def tree_obstacles(snapshot):
    return [(float(t['x']), float(t['y']), max(float(t.get('radius_m', 0.0)), 0.0)) for t in snapshot.get('trees', [])]


def nearest_traj_xy(traj, elapsed):
    idx = int(np.argmin(np.abs(traj['time_sec'].to_numpy() - elapsed)))
    row = traj.iloc[idx]
    return np.array([float(row.x), float(row.y)])


def build_gp_visit(candidates, observations, obs_count):
    gp = ExplorationGP(
        candidates[:, :2], config.GP_ELL, config.GP_SIGMA_S, config.GP_SIGMA_N, config.GP_BETA,
        config.GP_OBSERVATION_MODEL, config.GP_OBSERVATION_FOOTPRINT_RADIUS, config.GP_OBSERVATION_FOOTPRINT_SIGMA,
    )
    visited = VisitedMap(gp.observation_model, saturation=config.RRT_VISIT_SATURATION, decay_halflife_observations=config.RRT_VISIT_DECAY_HALFLIFE_OBS)
    n = max(0, min(int(obs_count), len(observations)))
    for _, row in observations.iloc[:n].iterrows():
        xy = np.array([float(row.x), float(row.y)])
        gp.add_observation(xy, float(row.point_score))
        visited.add_observation(xy)
    return gp, visited


def normalize(values):
    values = np.asarray(values, dtype=float); r = float(np.ptp(values))
    return np.zeros_like(values) if r <= 1e-12 else (values - float(np.min(values))) / r


def run_rrt_trace(start_xy, candidates, gp, visited, obstacles, seed):
    np.random.seed(seed)
    ucb_values = gp.ucb_grid()
    x_min, x_max, y_min, y_max = BOUNDS
    lower_bounds = np.array([x_min, y_min], dtype=float); upper_bounds = np.array([x_max, y_max], dtype=float)
    best_candidate = candidates[int(np.argmax(ucb_values)), :2]
    fantasy_gp = FantasyGP(gp)
    root = RRTNode(xy=np.asarray(start_xy[:2], dtype=float).copy(), parent=None, fantasy=fantasy_gp.root(), visit=visited.root())
    nodes = [root]
    for _ in range(int(config.RRT_N_ITER)):
        if np.random.rand() < config.RRT_GOAL_BIAS:
            sample = best_candidate + np.random.randn(2) * config.RRT_STEP_SIZE * 0.5
        else:
            sample = np.array([np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max)])
        sample = np.clip(sample, lower_bounds, upper_bounds)
        expandable = [node for node in nodes if node.depth < config.RRT_HORIZON]
        if not expandable: break
        nearest = _nearest_node(expandable, sample)
        direction = sample - nearest.xy; distance = float(np.linalg.norm(direction))
        if distance < 1e-6: continue
        new_xy = nearest.xy + (direction / distance) * min(config.RRT_STEP_SIZE, distance)
        if not _is_collision_free(nearest.xy, new_xy, obstacles): continue
        edge_cost = float(np.linalg.norm(new_xy - nearest.xy))
        node_ucb = _ucb_at(new_xy, candidates, ucb_values)
        node_fantasy = nearest.fantasy.observe_at(new_xy)
        node_visit = nearest.visit.observe_at(new_xy)
        discount = float(config.RRT_FUTURE_DISCOUNT) ** nearest.depth
        nodes.append(RRTNode(
            xy=new_xy, parent=nearest,
            ucb_accum=nearest.ucb_accum + discount * node_ucb,
            info_gain_accum=nearest.info_gain_accum + discount * node_fantasy.gain,
            novelty_accum=nearest.novelty_accum + discount * node_visit.novelty,
            revisit_accum=nearest.revisit_accum + discount * node_visit.revisit,
            cost=nearest.cost + edge_cost, depth=nearest.depth + 1,
            fantasy=node_fantasy, visit=node_visit,
        ))
    leaves = _terminal_nodes(nodes, config.RRT_HORIZON)
    id_to_index = {id(node): i for i, node in enumerate(nodes)}
    raw = pd.DataFrame({
        'node_index': [id_to_index[id(node)] for node in leaves],
        'x': [node.xy[0] for node in leaves], 'y': [node.xy[1] for node in leaves],
        'ucb_accum': [node.ucb_accum for node in leaves], 'info_gain_accum': [node.info_gain_accum for node in leaves],
        'novelty_accum': [node.novelty_accum for node in leaves], 'revisit_accum': [node.revisit_accum for node in leaves],
        'cost': [node.cost for node in leaves], 'depth': [node.depth for node in leaves],
    })
    raw['n_ucb'] = normalize(raw['ucb_accum']); raw['n_info'] = normalize(raw['info_gain_accum']); raw['n_novelty'] = normalize(raw['novelty_accum'])
    raw['n_cost'] = normalize(raw['cost']); raw['n_revisit'] = normalize(raw['revisit_accum'])
    raw['utility'] = (W_UCB * raw['n_ucb'] + W_INFO * raw['n_info'] + W_NOV * raw['n_novelty']) / WEIGHT_SUM
    raw['score'] = raw['utility'] - W_COST * raw['n_cost'] - W_REV * raw['n_revisit']
    raw['score_norev'] = raw['utility'] - W_COST * raw['n_cost']
    raw = raw.sort_values('score', ascending=False).reset_index(drop=True)
    best_leaf = nodes[int(raw.iloc[0]['node_index'])]
    best_path = _path_from_leaf(best_leaf)
    alt_row = raw.sort_values('score_norev', ascending=False).iloc[0]
    alt_path = _path_from_leaf(nodes[int(alt_row['node_index'])])
    top_paths = []
    for _, row in raw.head(8).iterrows():
        leaf = nodes[int(row['node_index'])]
        top_paths.append((row, _path_from_leaf(leaf)))
    return {'nodes': nodes, 'leaves': raw, 'best_leaf': best_leaf, 'best_path': best_path,
            'alt_row': alt_row, 'alt_path': alt_path, 'top_paths': top_paths,
            'ucb_values': ucb_values, 'mean': gp.mean_.copy(),
            'std': np.sqrt(np.maximum(np.diag(gp.cov_), 0.0)),
            'visit_mass': visited.visit_mass.copy()}


# ── Desenho: raster suave (imshow bilinear) igual ao vídeo ────────────────
def field_to_grid(candidates, values):
    xs = np.unique(candidates[:, 0]); ys = np.unique(candidates[:, 1])
    g = np.full((len(ys), len(xs)), np.nan)
    xi = {float(x): i for i, x in enumerate(xs)}; yi = {float(y): i for i, y in enumerate(ys)}
    for (px, py), v in zip(candidates[:, :2], values):
        g[yi[float(py)], xi[float(px)]] = v
    return np.nan_to_num(g), (xs[0], xs[-1], ys[0], ys[-1])


def style_axes(ax):
    ax.set_xlim(BOUNDS[0] - 1, BOUNDS[1] + 1); ax.set_ylim(BOUNDS[2] - 1, BOUNDS[3] + 1)
    ax.set_aspect('equal'); ax.set_facecolor(PANEL)
    ax.grid(True, color=GRID, alpha=0.6, lw=0.6)
    ax.tick_params(colors=MUT, labelsize=8)
    for s in ax.spines.values():
        s.set_color(GRID)


def draw_field(ax, candidates, values, cmap, title=None, vmin=None, vmax=None):
    g, extent = field_to_grid(candidates, values)
    vmin = float(np.min(values)) if vmin is None else vmin
    vmax = float(np.max(values)) if vmax is None else vmax
    im = ax.imshow(g, origin='lower', extent=extent, cmap=cmap, vmin=vmin, vmax=vmax,
                   interpolation='bilinear', aspect='equal', zorder=0)
    style_axes(ax)
    if title:
        ax.set_title(title, color=INK, fontsize=14, pad=8, weight='bold', loc='left')
    return im


def add_colorbar(fig, im, ax, label):
    cb = fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02)
    cb.set_label(label, color=MUT, fontsize=8)
    cb.ax.tick_params(colors=MUT, labelsize=7)
    cb.outline.set_edgecolor(GRID)
    return cb


def draw_overlays(ax, gt, snapshot, traj_until=None, drone_xy=None, legend=False):
    ax.scatter(gt['x'], gt['y'], marker='x', c=INK, s=34, lw=1.4, alpha=0.9, zorder=5, label='GT')
    trees = snapshot.get('trees', [])
    if trees:
        ax.scatter([t['x'] for t in trees], [t['y'] for t in trees], s=48, facecolors='none',
                   edgecolors=GREEN, lw=1.5, zorder=6, label='mapeada')
    if traj_until is not None and len(traj_until):
        ax.plot(traj_until['x'], traj_until['y'], color=INK, lw=1.2, alpha=0.40, zorder=4, label='trajetória')
    if drone_xy is not None:
        ax.scatter([drone_xy[0]], [drone_xy[1]], s=140, marker='o', color=DRONE, edgecolor=BG,
                   lw=1.6, zorder=13, label='drone (agora)')
    if legend:
        leg = ax.legend(loc='lower left', fontsize=8, framealpha=0.85, facecolor=PANEL, edgecolor=GRID)
        for t in leg.get_texts():
            t.set_color(INK)


def draw_plan(ax, drone_xy, best_path, commit=COMMIT, label_wp=True):
    """Committed (executado, ~commit wp) sólido e numerado; intenção tracejada até a estrela."""
    bp = np.asarray(best_path)
    if not len(bp):
        return
    full = np.vstack([drone_xy, bp])
    ncommit = min(commit, len(bp))
    comm = full[:ncommit + 1]
    ax.plot(comm[:, 0], comm[:, 1], color='#0b0e11', lw=6.5, alpha=0.9, zorder=8)
    ax.plot(comm[:, 0], comm[:, 1], color=YELLOW, lw=3.6, alpha=1.0, zorder=9, solid_capstyle='round')
    if len(bp) > ncommit:
        tail = full[ncommit:]
        ax.plot(tail[:, 0], tail[:, 1], color='#0b0e11', lw=4.0, alpha=0.65, zorder=6)
        ax.plot(tail[:, 0], tail[:, 1], color=YELLOW, lw=2.0, alpha=0.85, ls=(0, (4, 3)), zorder=7)
    for i in range(ncommit):
        wp = bp[i]
        ax.scatter([wp[0]], [wp[1]], s=165, color=YELLOW, edgecolor='#0b0e11', lw=1.4, zorder=11)
        if label_wp:
            ax.text(wp[0], wp[1], str(i + 1), fontsize=9, weight='bold', ha='center', va='center', color='#0b0e11', zorder=12)
    ax.scatter([bp[-1, 0]], [bp[-1, 1]], s=250, marker='*', color=YELLOW, edgecolor=INK, lw=1.2, zorder=11)
    ax.scatter([drone_xy[0]], [drone_xy[1]], s=150, marker='o', color=DRONE, edgecolor=BG, lw=1.6, zorder=13)


def caption_box(ax, lines, loc='upper left'):
    """Small translucent legend box in axes coords (avoids colorbar collisions)."""
    n = len(lines)
    h = 0.052 * n + 0.03
    x0, y0 = (0.015, 0.985 - h) if 'upper' in loc else (0.015, 0.015)
    ax.add_patch(Rectangle((x0, y0), 0.335, h, transform=ax.transAxes,
                           facecolor=BG, alpha=0.74, edgecolor=GRID, lw=0.8, zorder=20))
    for i, (txt, col) in enumerate(lines):
        ax.text(x0 + 0.018, y0 + h - 0.032 - i * 0.052, txt, transform=ax.transAxes,
                color=col, fontsize=9.5, va='center', zorder=21)


def select_spread(trace, n_top=3, n_mid=2, n_bot=3):
    """Escolhe top-N melhores + N medianos + N piores das folhas (por score)."""
    leaves = trace['leaves']; nodes = trace['nodes']; n = len(leaves)
    picks = []; used = set()

    def add(i, group):
        if 0 <= i < n and i not in used:
            used.add(i)
            row = leaves.iloc[i]
            picks.append({'rank': i, 'group': group, 'row': row,
                          'path': _path_from_leaf(nodes[int(row['node_index'])])})
    for i in range(n_top):
        add(i, 'top')
    c = n // 2
    for i in range(c - n_mid // 2, c - n_mid // 2 + n_mid):
        add(i, 'mid')
    for i in range(n - n_bot, n):
        add(i, 'bot')
    return picks


def draw_rrt_tree(ax, trace, picks, drone_xy):
    nodes = trace['nodes']
    segments = [[node.parent.xy, node.xy] for node in nodes if node.parent is not None]
    ax.add_collection(LineCollection(segments, colors=MUT, linewidths=0.45, alpha=0.18, zorder=2))
    style = {'top': dict(lw=1.9, alpha=0.9, ls='-'),
             'mid': dict(lw=1.6, alpha=0.8, ls='-'),
             'bot': dict(lw=1.4, alpha=0.75, ls=(0, (4, 3)))}
    for p in picks:
        if p['rank'] == 0:
            continue  # #1 executado é desenhado por draw_plan
        pts = np.asarray(p['path'])
        if len(pts) == 0:
            continue
        col = GROUP_COLORS[p['group']]
        ax.plot(pts[:, 0], pts[:, 1], color='#0b0e11', lw=style[p['group']]['lw'] + 2.2, alpha=0.5, zorder=3)
        ax.plot(pts[:, 0], pts[:, 1], color=col, zorder=4, **style[p['group']])
        ax.scatter([pts[-1, 0]], [pts[-1, 1]], s=110, color=col, edgecolor=BG, lw=0.9, zorder=6)
        ax.text(pts[-1, 0], pts[-1, 1], str(p['rank'] + 1), fontsize=7.5, weight='bold',
                ha='center', va='center', color=BG, zorder=7)
    draw_plan(ax, drone_xy, trace['best_path'])


def score_formula_text():
    return (r'$S(\tau)=\frac{0.40\,n(\Sigma UCB)+0.60\,n(\Sigma \Delta Tr)+0.35\,n(\Sigma nov)}{1.35}$'
            '\n' r'$\qquad -\,0.30\,n(cost)\;-\;0.45\,n(revisit)$')


def add_header(fig, title, subtitle='', slide_no=None):
    fig.text(0.035, 0.955, title, fontsize=23, weight='bold', color=INK, ha='left', va='top')
    if subtitle:
        fig.text(0.035, 0.905, subtitle, fontsize=11.5, color=MUT, ha='left', va='top')
    if slide_no is not None:
        fig.text(0.965, 0.955, f'{slide_no:02d}', fontsize=13, color=MUT, ha='right', va='top')


def new_fig():
    return plt.figure(figsize=(16, 9), facecolor=BG)


def save_slide(fig, idx, name):
    path = SLIDES / f'{idx:02d}_{name}.png'
    fig.savefig(path, dpi=150, facecolor=BG)
    plt.close(fig)
    return path


# ── Slides de abertura ────────────────────────────────────────────────────
def make_title_slide(idx, gt, gp_end, candidates, traj, subtitle):
    fig = new_fig()
    add_header(fig, 'Do campo do GP ao cérebro do drone', subtitle, idx)
    ax = fig.add_axes([0.05, 0.11, 0.55, 0.72])
    std = np.sqrt(np.maximum(np.diag(gp_end.cov_), 0.0))
    im = draw_field(ax, candidates, std, CMAP_STD, 'Incerteza σ(x) ao fim da missão', vmin=0.0)
    draw_overlays(ax, gt, {'trees': []}, traj, None, legend=False)
    ax.plot(traj['x'], traj['y'], color=INK, lw=1.0, alpha=0.35, zorder=4)
    add_colorbar(fig, im, ax, 'σ')
    ax2 = fig.add_axes([0.64, 0.12, 0.32, 0.68]); ax2.axis('off'); ax2.set_xlim(0, 1); ax2.set_ylim(0, 1)
    ax2.text(0, 0.98, 'A apresentação em duas partes', fontsize=16, weight='bold', color=INK, va='top')
    ax2.text(0.0, 0.87,
             '1 · O drone constrói um GP (média μ e\n     incerteza σ) — é o que o vídeo\n     gp_propagation.mp4 mostra.\n\n'
             '2 · Entramos no “cérebro” em instantes-\n     chave: por que ele escolhe cada\n     direção e como o score S(τ) age.',
             fontsize=12.5, color='#cbd5e1', va='top', linespacing=1.5)
    ax2.text(0, 0.40, 'Score de um ramo da RRT', fontsize=14, weight='bold', color=INK, va='top')
    ax2.text(0, 0.31, score_formula_text(), fontsize=13, color=INK, va='top',
             bbox=dict(boxstyle='round,pad=.6', facecolor=PANEL, edgecolor=GRID))
    ax2.text(0, 0.07, f'novelty = e^(−revisita)   ·   meia-vida = 450 obs   ·   commit = {COMMIT} waypoints',
             fontsize=10.5, color=MUT, va='top')
    return save_slide(fig, idx, '00_titulo')


def make_gp_propagation_slide(idx, gt, gp_end, candidates, traj):
    fig = new_fig()
    add_header(fig, 'Parte 1 — o que o drone constrói: propagação do GP',
               'mesma lógica Kalman do planner online · μ = point score esperado · σ = desvio-padrão (entra no UCB)', idx)
    mean = gp_end.mean_
    std = np.sqrt(np.maximum(np.diag(gp_end.cov_), 0.0))
    axm = fig.add_axes([0.055, 0.10, 0.42, 0.74])
    im1 = draw_field(axm, candidates, mean, CMAP_MEAN, 'Média posterior  μ(x)', vmin=0.0, vmax=float(np.percentile(mean, 99)))
    draw_overlays(axm, gt, {'trees': []}, traj, None)
    add_colorbar(fig, im1, axm, 'point score')
    axs = fig.add_axes([0.545, 0.10, 0.42, 0.74])
    im2 = draw_field(axs, candidates, std, CMAP_STD, 'Incerteza  σ(x)', vmin=0.0)
    draw_overlays(axs, gt, {'trees': []}, traj, None)
    add_colorbar(fig, im2, axs, 'σ')
    fig.text(0.055, 0.055,
             'Onde μ é alto o drone espera árvores (utilidade); onde σ é alto ele ainda não olhou (informação). '
             'O UCB = μ + β·σ combina os dois — é o fundo dos mapas a seguir.',
             fontsize=12, color='#cbd5e1')
    return save_slide(fig, idx, '01_gp_propagacao')


def make_decision_slide(idx):
    fig = new_fig()
    add_header(fig, 'Parte 2 — como o cérebro decide (e por que o waypoint ≠ destino)',
               'a cada replan a RRT cresce ~300 ramos; cada folha recebe um score S(τ)', idx)
    ax = fig.add_axes([0.05, 0.08, 0.6, 0.78]); ax.axis('off'); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    steps = [
        ('GP', '#38bdf8', 'μ e σ → UCB = μ + β·σ  (utilidade + incerteza)'),
        ('Fantasy GP', '#4ade80', 'quanto cada ramo reduziria a incerteza (info)'),
        ('VisitedMap', '#a3e635', 'novelty premia o novo · revisit pune o já-coberto (decay 450 obs)'),
        ('RRT', YELLOW, f'compara as folhas por S(τ) e escolhe a melhor'),
        ('Commit', RED, f'executa só os {COMMIT} primeiros waypoints e replaneja'),
    ]
    y = 0.9
    for name, col, desc in steps:
        ax.add_patch(Rectangle((0.0, y - 0.055), 0.2, 0.085, facecolor=col, edgecolor='none', alpha=0.9))
        ax.text(0.1, y - 0.013, name, fontsize=12.5, weight='bold', ha='center', va='center', color='#0b0e11')
        ax.text(0.23, y - 0.013, desc, fontsize=12.5, color='#cbd5e1', va='center')
        y -= 0.135
    ax.text(0.0, 0.20, score_formula_text(), fontsize=15, color=INK, va='top',
            bbox=dict(boxstyle='round,pad=.6', facecolor=PANEL, edgecolor=GRID))
    axr = fig.add_axes([0.67, 0.1, 0.30, 0.74]); axr.axis('off'); axr.set_xlim(0, 1); axr.set_ylim(0, 1)
    axr.text(0, 0.98, 'Por que o drone não vai\naté a estrela ★', fontsize=15, weight='bold', color=INK, va='top')
    axr.text(0, 0.80,
             f'A estrela é a folha da RRT: a intenção\ncompleta (até {config.RRT_HORIZON} wp ≈ {config.RRT_HORIZON*STEP:.0f} m).\n\n'
             f'Mas o drone só se compromete com os\n{COMMIT} primeiros waypoints (≈ {COMMIT*STEP:.0f} m, linha\nsólida amarela). Ao chegar no {COMMIT}º ele\nreplaneja com as novas observações.\n\n'
             'Como o GP mudou, a nova folha costuma\napontar para outro lado — por isso a\nestrela indica direção, não destino.',
             fontsize=12, color='#cbd5e1', va='top', linespacing=1.5)
    axr.add_patch(Rectangle((0, 0.02), 0.06, 0.03, facecolor=YELLOW, edgecolor='none'))
    axr.text(0.09, 0.035, f'sólido = {COMMIT} wp comprometidos (executa)', fontsize=10.5, color=INK, va='center')
    axr.add_patch(Rectangle((0, -0.05), 0.06, 0.006, facecolor=YELLOW, edgecolor='none', alpha=0.6))
    axr.text(0.09, -0.045, 'tracejado = intenção (quase nunca voa)', fontsize=10.5, color=MUT, va='center')
    return save_slide(fig, idx, '02_decisao')


# ── Slides por momento ─────────────────────────────────────────────────────
def slide_gp_state(idx, moment_name, rec, gt, snapshot, traj, candidates, trace, drone_xy):
    fig = new_fig()
    add_header(fig, f'{moment_name}: estado do GP',
               f't={rec.elapsed:.0f}s · obs={rec.obs_count} · árvores no mapa={snapshot.get("tree_count",0)} · footprint 3.0 m', idx)
    ax = fig.add_axes([0.055, 0.10, 0.60, 0.75]); traj_until = traj[traj['time_sec'] <= rec.elapsed]
    im = draw_field(ax, candidates, trace['ucb_values'], CMAP_UCB, 'UCB = μ + β·σ (fundo claro = atrai)')
    draw_overlays(ax, gt, snapshot, traj_until, drone_xy, legend=False)
    if len(trace['best_path']):
        bp = np.asarray(trace['best_path'])
        ax.annotate('', xy=(bp[0, 0], bp[0, 1]), xytext=(drone_xy[0], drone_xy[1]),
                    arrowprops=dict(arrowstyle='->', color=YELLOW, lw=2.6))
    caption_box(ax, [('●  drone (agora)', DRONE), ('→  1º waypoint', YELLOW),
                     ('✕ GT   ◦ árvore mapeada', INK)])
    add_colorbar(fig, im, ax, 'UCB')
    axr = fig.add_axes([0.70, 0.12, 0.26, 0.70]); axr.axis('off'); axr.set_xlim(0, 1); axr.set_ylim(0, 1)
    stats = [('score log', f'{rec.logged_score:.3f}'), ('primeiro wp', f'[{rec.logged_first[0]:.1f}, {rec.logged_first[1]:.1f}]'),
             ('folha (leaf)', f'[{rec.logged_leaf[0]:.1f}, {rec.logged_leaf[1]:.1f}]'),
             ('revisit log', f'{rec.logged_revisit:.2f}'), ('novelty log', f'{rec.logged_novelty:.2f}')]
    y = 0.98; axr.text(0, y, 'Estado observado', fontsize=16, weight='bold', color=INK, va='top'); y -= 0.10
    for k, v in stats:
        axr.text(0, y, k, fontsize=11, color=MUT); axr.text(0.55, y, v, fontsize=13.5, weight='bold', color=INK); y -= 0.085
    axr.text(0, y - 0.02, 'Leitura', fontsize=16, weight='bold', color=INK, va='top'); y -= 0.13
    axr.text(0, y, '• fundo claro/alto = região que atrai\n• ponto branco = onde o drone está agora\n• círculo verde = árvore já no tree_mapper\n• seta = 1º waypoint escolhido',
             fontsize=11.5, color='#cbd5e1', va='top', linespacing=1.5)
    return save_slide(fig, idx, f'{rec.index:02d}_gp')


def slide_rrt_score(idx, moment_name, rec, gt, snapshot, candidates, trace, drone_xy):
    fig = new_fig()
    add_header(fig, f'{moment_name}: árvore RRT e score dos ramos',
               f'replay determinístico do estado do planner · nós={len(trace["nodes"])} · folhas={len(trace["leaves"])}', idx)
    picks = select_spread(trace)
    ax = fig.add_axes([0.045, 0.10, 0.52, 0.75])
    im = draw_field(ax, candidates, trace['ucb_values'], CMAP_UCB, 'RRT: top 3 · 2 medianos · piores 3')
    draw_rrt_tree(ax, trace, picks, drone_xy)
    add_colorbar(fig, im, ax, 'UCB')
    axf = fig.add_axes([0.60, 0.73, 0.37, 0.13]); axf.axis('off')
    axf.text(0, 0.95, score_formula_text(), fontsize=12, color=INK, va='top',
             bbox=dict(boxstyle='round,pad=.45', facecolor=PANEL, edgecolor=GRID))
    axt = fig.add_axes([0.60, 0.11, 0.37, 0.57]); axt.set_xlim(0, 1); axt.set_ylim(0, 1); axt.axis('off')
    headers = ['#', 'S', 'UCB', 'I', 'Nov', '−C', '−Rev']; xs = [0.02, 0.115, 0.24, 0.38, 0.52, 0.68, 0.84]
    for x, h in zip(xs, headers):
        axt.text(x, 0.975, h, fontsize=10.5, weight='bold', color=INK, ha='left')
    # linhas: top3 · gap · 2 medianos · gap · piores3
    display = []
    for g in ('top', 'mid', 'bot'):
        if display:
            display.append(('gap', None))
        display += [('row', p) for p in picks if p['group'] == g]
    n_leaves = len(trace['leaves'])
    y = 0.90
    for kind, p in display:
        if kind == 'gap':
            axt.text(0.5, y - 0.005, f'⋯  ({n_leaves} folhas no total)  ⋯', fontsize=8.5, color=MUT, ha='center', va='center')
            y -= 0.052
            continue
        row = p['row']; rank = p['rank']; gcol = GROUP_COLORS[p['group']]
        bg = '#3a2f12' if rank == 0 else PANEL
        axt.add_patch(Rectangle((0.0, y - 0.036), 0.98, 0.066, facecolor=bg, edgecolor=GRID, lw=.6))
        axt.add_patch(Rectangle((0.0, y - 0.036), 0.008, 0.066, facecolor=gcol, edgecolor='none'))
        lbl = f'{rank+1}' + ('★' if rank == 0 else '')
        axt.text(xs[0], y, lbl, fontsize=10, weight='bold', va='center', color=gcol)
        axt.text(xs[1], y, f'{row.score:.3f}', fontsize=10, weight='bold', va='center', color=INK)
        for j, (val, key) in enumerate(zip([row.n_ucb, row.n_info, row.n_novelty, row.n_cost, row.n_revisit], ['UCB', 'I', 'Nov', '-C', '-Rev'])):
            x = xs[j + 2]
            axt.add_patch(Rectangle((x, y - 0.017), 0.115, 0.03, facecolor=GRID, edgecolor='none'))
            axt.add_patch(Rectangle((x, y - 0.017), 0.115 * float(val), 0.03, facecolor=BAR_COLORS[key], edgecolor='none'))
            axt.text(x + 0.057, y - 0.036, f'{val:.2f}', fontsize=7, ha='center', va='top', color=MUT)
        y -= 0.088
    axt.text(0.0, 0.005, '#1★ = executado · verde=top · azul=mediano · vermelho=pior', fontsize=9, color=MUT)
    return save_slide(fig, idx, f'{rec.index:02d}_rrt_score')


def slide_action(idx, moment_name, rec, gt, snapshot, traj, candidates, trace, drone_xy):
    fig = new_fig(); top = trace['leaves'].iloc[0]
    add_header(fig, f'{moment_name}: ação escolhida — commit vs intenção',
               f'S replay={top.score:.3f} · S log={rec.logged_score:.3f} · executa {COMMIT} wp (~{COMMIT*STEP:.0f} m), depois replaneja', idx)
    ax = fig.add_axes([0.055, 0.10, 0.62, 0.75]); traj_until = traj[traj['time_sec'] <= rec.elapsed]
    im = draw_field(ax, candidates, trace['std'], CMAP_STD, 'Incerteza σ + caminho executado', vmin=0.0)
    draw_overlays(ax, gt, snapshot, traj_until, None)
    draw_plan(ax, drone_xy, trace['best_path'])
    caption_box(ax, [('●  drone (agora)', DRONE),
                     (f'①②③  commit: {COMMIT} wp (~{COMMIT*STEP:.0f} m)', YELLOW),
                     ('★  intenção (folha RRT)', YELLOW)])
    add_colorbar(fig, im, ax, 'σ')
    axr = fig.add_axes([0.71, 0.14, 0.25, 0.66]); axr.axis('off'); axr.set_xlim(0, 1); axr.set_ylim(0, 1)
    axr.text(0, 0.98, 'Score #1', fontsize=17, weight='bold', color=INK, va='top')
    rows = [('utility', top.utility, 'utility'), ('+ UCB', top.n_ucb, 'UCB'), ('+ info', top.n_info, 'I'),
            ('+ novelty', top.n_novelty, 'Nov'), ('− cost', top.n_cost, '-C'), ('− revisit', top.n_revisit, '-Rev')]
    y = 0.86
    for label, val, key in rows:
        axr.text(0, y, label, fontsize=11, color='#cbd5e1', va='center')
        axr.add_patch(Rectangle((0.44, y - 0.025), 0.48, 0.05, facecolor=GRID, edgecolor='none'))
        axr.add_patch(Rectangle((0.44, y - 0.025), 0.48 * float(val), 0.05, facecolor=BAR_COLORS[key], edgecolor='none'))
        axr.text(0.95, y, f'{val:.2f}', fontsize=10, ha='right', va='center', color=INK); y -= 0.105
    axr.text(0, y - 0.02, f'S = {top.score:.3f}', fontsize=23, weight='bold', color=INK, va='top')
    axr.text(0, 0.06, f'Amarelo sólido = {COMMIT} wp que o\ndrone realmente voa.\nTracejado até ★ = intenção que\nserá recalculada no próximo replan.',
             fontsize=10.5, color=MUT, va='top', linespacing=1.4)
    return save_slide(fig, idx, f'{rec.index:02d}_acao')


def slide_why_descend(idx, rec, gt, snapshot, traj, candidates, trace, drone_xy):
    """Por que o drone desceu ao Sul entre M1 e M2: commit ≠ direção da estrela."""
    fig = new_fig()
    path = np.asarray(rec.logged_path, dtype=float)
    add_header(fig, 'Entre M1 e M2: por que o drone desceu ao Sul?',
               f'plano real do log em t={rec.elapsed:.0f}s · estrela ao Norte, mas os {COMMIT} wp executados descem · árvore truncada em {rec.logged_depth} passos', idx)
    ax = fig.add_axes([0.055, 0.10, 0.60, 0.75])
    im = draw_field(ax, candidates, trace['ucb_values'], CMAP_UCB, 'UCB no instante do plano (Sul = alto → atrai)')
    traj_until = traj[traj['time_sec'] <= rec.elapsed]
    draw_overlays(ax, gt, snapshot, traj_until, None)
    if len(path):
        nc = min(COMMIT, len(path))
        # conector do drone real ao 1º waypoint do plano
        ax.plot([drone_xy[0], path[0, 0]], [drone_xy[1], path[0, 1]], color=MUT, lw=1.0, ls=':', alpha=0.7, zorder=6)
        # committed: pontos 1..nc (executados) — descem ao Sul
        comm = path[:nc]
        ax.plot(comm[:, 0], comm[:, 1], color='#0b0e11', lw=6.5, alpha=0.9, zorder=8)
        ax.plot(comm[:, 0], comm[:, 1], color=YELLOW, lw=3.6, zorder=9, solid_capstyle='round')
        # intenção: pontos nc-1..fim (tracejado até a estrela) — sobe ao Norte
        tail = path[nc - 1:]
        ax.plot(tail[:, 0], tail[:, 1], color='#0b0e11', lw=4.0, alpha=0.65, zorder=6)
        ax.plot(tail[:, 0], tail[:, 1], color=YELLOW, lw=2.0, alpha=0.9, ls=(0, (4, 3)), zorder=7)
        for i in range(nc):
            ax.scatter([path[i, 0]], [path[i, 1]], s=165, color=YELLOW, edgecolor='#0b0e11', lw=1.4, zorder=11)
            ax.text(path[i, 0], path[i, 1], str(i + 1), fontsize=9, weight='bold', ha='center', va='center', color='#0b0e11', zorder=12)
        ax.scatter([path[-1, 0]], [path[-1, 1]], s=260, marker='*', color=YELLOW, edgecolor=INK, lw=1.2, zorder=11)
        ax.scatter([drone_xy[0]], [drone_xy[1]], s=150, marker='o', color=DRONE, edgecolor=BG, lw=1.6, zorder=13)
        ax.annotate('★ alvo ao Norte\n(passo 8 — nunca voa)', xy=(path[-1, 0], path[-1, 1]),
                    xytext=(path[-1, 0] - 8.5, path[-1, 1] + 3.0), fontsize=10.5, color=INK,
                    bbox=dict(boxstyle='round,pad=.3', facecolor=BG, edgecolor=GRID, alpha=0.85),
                    arrowprops=dict(arrowstyle='->', color=INK, lw=1.2))
    add_colorbar(fig, im, ax, 'UCB')
    caption_box(ax, [('●  drone (agora)', DRONE), (f'①②③  {COMMIT} wp executados → Sul', YELLOW),
                     ('★  alvo da RRT → Norte (não voa)', YELLOW)])
    axr = fig.add_axes([0.69, 0.10, 0.28, 0.75]); axr.axis('off'); axr.set_xlim(0, 1); axr.set_ylim(0, 1)
    axr.text(0, 0.99, 'Ele seguiu os 3 waypoints', fontsize=15.5, weight='bold', color=INK, va='top')
    axr.text(0, 0.90,
             'No run inteiro: 0 commits descartados.\nO drone executou os 3 wp normalmente.',
             fontsize=11.5, color='#cbd5e1', va='top', linespacing=1.45)
    axr.text(0, 0.76, 'Só que os 3 wp já apontam ao Sul', fontsize=13.5, weight='bold', color=YELLOW, va='top')
    axr.text(0, 0.685,
             'A rota da RRT tem 8 passos e é curva:\nvarre a faixa Sul (σ/UCB alto) e só no\n8º passo chega à estrela ao Norte.\n'
             'Os 3 wp comprometidos são os 3\nprimeiros passos — todos descendo.',
             fontsize=11.5, color='#cbd5e1', va='top', linespacing=1.45)
    axr.text(0, 0.42, 'Dois fatores de truncamento', fontsize=13.5, weight='bold', color=INK, va='top')
    axr.text(0, 0.345,
             f'• Horizonte da árvore\n   RRT_HORIZON = {config.RRT_HORIZON} passos (~{config.RRT_HORIZON*STEP:.0f} m)\n   → trunca a intenção (a estrela).\n\n'
             f'• Commit\n   RRT_COMMITTED_WAYPOINTS = {COMMIT} (~{COMMIT*STEP:.0f} m)\n   → trunca a execução.',
             fontsize=11.5, color='#cbd5e1', va='top', linespacing=1.4)
    axr.text(0, 0.05,
             'Ao terminar os 3 wp (no Sul) ele\nreplaneja; com o GP novo, subir ao\nNorte deixa de compensar → M2.',
             fontsize=11, color=MUT, va='top', linespacing=1.4)
    return save_slide(fig, idx, f'{rec.index:02d}_why_descend')


def slide_revisit_counterfactual(idx, moment_name, rec, gt, snapshot, traj, candidates, trace, drone_xy):
    """Momento em que o termo −Rev vira a decisão: com −Rev vs sem −Rev."""
    fig = new_fig()
    exec_row = trace['leaves'].iloc[0]; alt_row = trace['alt_row']
    add_header(fig, f'{moment_name}',
               f't={rec.elapsed:.0f}s · obs={rec.obs_count} · revisit_accum log={rec.logged_revisit:.1f} (alto) · aqui o −Rev muda o rumo', idx)
    ax = fig.add_axes([0.055, 0.10, 0.60, 0.75])
    im = draw_field(ax, candidates, trace['visit_mass'], CMAP_REVISIT, 'Massa de revisita (brilho = já muito coberto)', vmin=0.0)
    traj_until = traj[traj['time_sec'] <= rec.elapsed]
    draw_overlays(ax, gt, snapshot, traj_until, None)
    alt = np.asarray(trace['alt_path'])
    if len(alt):
        fa = np.vstack([drone_xy, alt])
        ax.plot(fa[:, 0], fa[:, 1], color=RED, lw=2.6, alpha=0.9, ls=(0, (5, 3)), zorder=7)
        ax.scatter([alt[-1, 0]], [alt[-1, 1]], s=210, marker='X', color=RED, edgecolor=INK, lw=1.0, zorder=8)
    draw_plan(ax, drone_xy, trace['best_path'])
    caption_box(ax, [('━ com −Rev: executado → área nova', YELLOW),
                     ('╌ sem −Rev: repassa terreno já varrido (✕)', RED),
                     ('●  drone (agora)', DRONE)])
    add_colorbar(fig, im, ax, 'visita acumulada')
    axr = fig.add_axes([0.69, 0.11, 0.28, 0.74]); axr.axis('off'); axr.set_xlim(0, 1); axr.set_ylim(0, 1)
    axr.text(0, 0.98, 'O termo −Rev é decisivo', fontsize=15.5, weight='bold', color=INK, va='top')
    jump = float(np.hypot(exec_row['x'] - alt_row['x'], exec_row['y'] - alt_row['y']))

    def block(y0, title, color, row, use_rev):
        axr.text(0, y0, title, fontsize=12.5, weight='bold', color=color, va='top')
        util = float(row['utility']); ncost = float(row['n_cost']); nrev = float(row['n_revisit'])
        s = util - W_COST * ncost - (W_REV * nrev if use_rev else 0.0)
        lines = [f'utility = {util:.2f}', f'− cost   = −{W_COST*ncost:.2f}']
        if use_rev:
            lines.append(f'− revisit = −{W_REV*nrev:.2f}   (n_rev={nrev:.2f})')
        else:
            lines.append(f'(revisit ignorado, n_rev={nrev:.2f})')
        lines.append(f'S = {s:.3f}')
        axr.text(0.02, y0 - 0.055, '\n'.join(lines), fontsize=11, color='#cbd5e1', va='top', linespacing=1.45)
        return s

    s_exec = block(0.86, 'Ramo executado (com −Rev)', YELLOW, exec_row, True)
    s_alt_norev = block(0.55, 'Preferido se −Rev = 0', RED, alt_row, False)
    s_alt_withrev = float(alt_row['utility']) - W_COST * float(alt_row['n_cost']) - W_REV * float(alt_row['n_revisit'])
    axr.text(0, 0.24,
             f'Sem o −Rev, o topo saltaria {jump:.0f} m\npara um ramo que repassa terreno já\nvarrido (n_rev={float(alt_row["n_revisit"]):.2f} vs {float(exec_row["n_revisit"]):.2f}).\n\n'
             f'O −Rev derruba esse ramo de\nS={s_alt_norev:.3f} para S={s_alt_withrev:.3f}, abaixo\ndo ramo novo (S={s_exec:.3f}).\n\n'
             f'É o que impede o drone de\nre-arar terreno velho.',
             fontsize=11, color=INK, va='top', linespacing=1.45)
    return save_slide(fig, idx, f'{rec.index:02d}_revisit')


def count_log_events():
    txt = (RUN / 'gaussian_feeder.log').read_text(errors='replace')
    return (len(re.findall(r'RRT path fresh', txt)),
            len(re.findall(r'RRT path precomputed', txt)),
            len(re.findall(r'Mantendo committed', txt)),
            len(re.findall(r'Descartando committed', txt)))


def make_summary_slide(idx, gt, gp_end, candidates, traj, snaps, records, observations):
    fig = new_fig()
    add_header(fig, 'Resumo do run', RUN_LABEL, idx)
    gt_n = len(gt)
    final_snap = snapshot_at(snaps, 1e9)
    trees = int((RUN_SUMMARY.get('tree_count_final_by_method', {}) or {}).get('tree_mapper', final_snap.get('tree_count', 0)))
    fresh, precomp, kept, disc = count_log_events()
    metrics = [
        ('árvores mapeadas', f'{trees}/{gt_n}', trees >= gt_n and gt_n > 0),
        ('waypoints enviados', str(RUN_SUMMARY.get('waypoint_count', '—')), False),
        ('replanejamentos RRT', f'{len(records)} ({fresh} fresh)', False),
        ('commits mantidos / descartados', f'{kept} / {disc}', disc == 0),
        ('observações no GP', str(len(observations)), False),
        ('distância XY', f'{RUN_SUMMARY.get("distance_xy_m", 0):.0f} m', False),
        ('duração', f'{RUN_SUMMARY.get("run_duration_sec", 0):.0f} s', False),
    ]
    ax = fig.add_axes([0.06, 0.12, 0.40, 0.70]); ax.axis('off'); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.text(0, 0.99, 'Métricas do run', fontsize=15, weight='bold', color=INK, va='top')
    y = 0.88
    for label, val, good in metrics:
        ax.text(0.02, y, label, fontsize=12, color='#cbd5e1', va='center')
        ax.text(0.98, y, val, fontsize=16, weight='bold', color=GREEN if good else INK, ha='right', va='center')
        ax.add_line(plt.Line2D([0.02, 0.98], [y - 0.05, y - 0.05], color=GRID, lw=1)); y -= 0.115
    ax2 = fig.add_axes([0.52, 0.12, 0.44, 0.72])
    std = np.sqrt(np.maximum(np.diag(gp_end.cov_), 0.0))
    im = draw_field(ax2, candidates, std, CMAP_STD, 'trajetória vs. ground-truth', vmin=0.0)
    draw_overlays(ax2, gt, final_snap, None, None, legend=True)
    ax2.plot(traj['x'], traj['y'], color=INK, lw=1.0, alpha=0.4, zorder=4)
    add_colorbar(fig, im, ax2, 'σ')
    fig.text(0.06, 0.055,
             f'{RUN_SUMMARY.get("finish_reason", "")} · {trees}/{gt_n} árvores mapeadas · {disc} commits descartados no run inteiro · {kept} mantidos após mudança no mapa.',
             fontsize=12, color='#cbd5e1')
    return save_slide(fig, idx, '99_resumo')


def _rev_decisiveness(trace):
    """Quão decisivo é o −Rev neste replan: salto (m) do #1 ao remover o termo."""
    L = trace['leaves']
    w = L.sort_values('score', ascending=False).iloc[0]
    wo = L.sort_values('score_norev', ascending=False).iloc[0]
    flips = int(w['node_index']) != int(wo['node_index'])
    jump = float(np.hypot(w['x'] - wo['x'], w['y'] - wo['y']))
    return jump if (flips and float(wo['n_revisit']) > 0.3) else 0.0


def _commit_star_divergence(rec):
    """Ângulo (graus) entre o rumo dos COMMIT wp e o rumo da estrela (folha)."""
    p = np.asarray(rec.logged_path, dtype=float)
    if len(p) < COMMIT + 2:
        return -1.0
    a = p[COMMIT - 1] - p[0]; b = p[-1] - p[0]
    na = np.linalg.norm(a); nb = np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return -1.0
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


def select_moments(records, cache):
    """Escolhe ~5 momentos espalhados no tempo + o momento em que o −Rev decide."""
    n = len(records)
    # −Rev decisivo (varredura em todos os replans)
    rev_idx, best = None, 4.0
    for rec in records:
        s = _rev_decisiveness(cache[rec.index][2])
        if s > best:
            best, rev_idx = s, rec.index
    # 5 momentos espalhados por tempo
    spread = sorted(set(min(n - 1, max(0, int(round(f * (n - 1))))) for f in (0.0, 0.25, 0.5, 0.75, 1.0)))
    # why_descend: maior divergência commit×estrela (entre os spread, não o 1º, não o rev)
    why_idx = None
    cands = [i for i in spread if i != rev_idx and i != spread[0]]
    if cands:
        wi = max(cands, key=lambda i: _commit_star_divergence(records[i]))
        if _commit_star_divergence(records[wi]) > 70.0:
            why_idx = wi
    ordered = sorted(set(spread) | ({rev_idx} if rev_idx is not None else set()))
    moments = []; k = 0
    for i in ordered:
        if i == rev_idx:
            moments.append(('R · quando o termo −Rev decide', i, 'revisit', None))
        else:
            extra = 'why_descend' if i == why_idx else None
            phase = 'início' if i == ordered[0] else ('fechamento' if i == ordered[-1] else '')
            label = f'M{k} · t={records[i].elapsed:.0f}s' + (f' ({phase})' if phase else '')
            moments.append((label, i, 'normal', extra)); k += 1
    return moments


def main(run_dir):
    configure(run_dir)
    gt = read_gt(); records = parse_plans()
    observations = pd.read_csv(RUN / 'gaussian_feeder/observations.csv')
    traj = pd.read_csv(RUN / 'trajectory.csv')
    snaps = load_snapshots()
    candidates = generate_waypoint_candidates(BOUNDS, resolution=config.WAYPOINT_GRID_RESOLUTION, height=config.FLIGHT_HEIGHT)
    if not records:
        print('AVISO: nenhum plano RRT encontrado no log; deck não gerado para', RUN)
        return None

    gp_end, _ = build_gp_visit(candidates, observations, len(observations))

    # trace de cada replan uma única vez (reusado na seleção e na renderização)
    cache = {}
    for rec in records:
        snapshot = snapshot_at(snaps, rec.elapsed)
        drone_xy = nearest_traj_xy(traj, rec.elapsed)
        gp, visited = build_gp_visit(candidates, observations, rec.obs_count)
        trace = run_rrt_trace(drone_xy, candidates, gp, visited, tree_obstacles(snapshot), rng_seed_base + rec.index)
        cache[rec.index] = (snapshot, drone_xy, trace)

    moments = select_moments(records, cache)
    trees = int((RUN_SUMMARY.get('tree_count_final_by_method', {}) or {}).get('tree_mapper', snapshot_at(snaps, 1e9).get('tree_count', 0)))
    subtitle = f'{RUN_LABEL} · {trees}/{len(gt)} árvores · {RUN_SUMMARY.get("distance_xy_m", 0):.0f} m · {RUN_SUMMARY.get("run_duration_sec", 0):.0f} s'

    slide_paths = []; idx = 1; manifest = []
    slide_paths.append(make_title_slide(idx, gt, gp_end, candidates, traj, subtitle)); idx += 1
    slide_paths.append(make_gp_propagation_slide(idx, gt, gp_end, candidates, traj)); idx += 1
    slide_paths.append(make_decision_slide(idx)); idx += 1

    for name, rec_i, kind, extra in moments:
        rec = records[rec_i]; snapshot, drone_xy, trace = cache[rec_i]
        manifest.append({'moment': name, 'record_index': rec.index, 'kind': kind, 'elapsed_sec': rec.elapsed,
                         'obs_count': rec.obs_count, 'tree_count_snapshot': snapshot.get('tree_count', 0),
                         'drone_x': float(drone_xy[0]), 'drone_y': float(drone_xy[1]),
                         'logged_score': rec.logged_score, 'replay_score': float(trace['leaves'].iloc[0]['score'])})
        slide_paths.append(slide_gp_state(idx, name, rec, gt, snapshot, traj, candidates, trace, drone_xy)); idx += 1
        slide_paths.append(slide_rrt_score(idx, name, rec, gt, snapshot, candidates, trace, drone_xy)); idx += 1
        if kind == 'revisit':
            slide_paths.append(slide_revisit_counterfactual(idx, name, rec, gt, snapshot, traj, candidates, trace, drone_xy)); idx += 1
        else:
            slide_paths.append(slide_action(idx, name, rec, gt, snapshot, traj, candidates, trace, drone_xy)); idx += 1
        if extra == 'why_descend':
            slide_paths.append(slide_why_descend(idx, rec, gt, snapshot, traj, candidates, trace, drone_xy)); idx += 1

    slide_paths.append(make_summary_slide(idx, gt, gp_end, candidates, traj, snaps, records, observations)); idx += 1

    pdf_path = OUT / 'rrt_gp_presentation_deck.pdf'
    with PdfPages(pdf_path) as pdf:
        for png in slide_paths:
            fig = new_fig(); ax = fig.add_axes([0, 0, 1, 1]); ax.imshow(plt.imread(png)); ax.axis('off'); pdf.savefig(fig, dpi=150, facecolor=BG); plt.close(fig)
    pptx_path = None
    if PPTX_AVAILABLE:
        prs = Presentation(); prs.slide_width = Inches(16); prs.slide_height = Inches(9); blank = prs.slide_layouts[6]
        for png in slide_paths:
            slide = prs.slides.add_slide(blank); slide.shapes.add_picture(str(png), 0, 0, width=prs.slide_width, height=prs.slide_height)
        pptx_path = OUT / 'rrt_gp_presentation_deck.pptx'; prs.save(pptx_path)
    (OUT / 'manifest.json').write_text(json.dumps({
        'run': str(RUN), 'slide_count': len(slide_paths), 'slides': [str(p) for p in slide_paths],
        'pdf': str(pdf_path), 'pptx': str(pptx_path) if pptx_path else None, 'moments': manifest,
        'formula': 'S=(0.40*n(UCB)+0.60*n(info)+0.35*n(novelty))/1.35 -0.30*n(cost)-0.45*n(revisit)',
        'note': 'Dark video-style raster deck. RRT trees are deterministic visual replays reconstructed from the logged GP/map state; the drone marker sits on the real trajectory at each replan time.'}, indent=2))
    print('OUT', OUT); print('slides', len(slide_paths)); print('pdf', pdf_path); print('pptx', pptx_path)
    return OUT


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='Gera o deck GP+RRT para um run (rep). Aceita 1+ dirs de rep ou um dir de benchmark (processa todos os *rep*).')
    ap.add_argument('runs', nargs='*', default=[DEFAULT_RUN], help='pasta(s) de rep OU pasta de benchmark')
    args = ap.parse_args()
    targets = []
    for r in args.runs:
        p = Path(r)
        reps = sorted(d for d in p.glob('*rep*') if d.is_dir() and (d / 'gaussian_feeder.log').exists())
        if (p / 'gaussian_feeder.log').exists():
            targets.append(p)
        elif reps:
            targets.extend(reps)
        else:
            print('AVISO: nada com gaussian_feeder.log em', p)
    for t in targets:
        print('\n===== gerando deck para', t, '=====')
        try:
            main(str(t))
        except Exception as e:
            import traceback; print('ERRO em', t, ':', e); traceback.print_exc()
