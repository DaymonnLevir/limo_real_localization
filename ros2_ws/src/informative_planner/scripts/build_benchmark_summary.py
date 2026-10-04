"""Deck agregado de um benchmark (N reps): métricas médias ± desvio + trajetórias.

Uso:
  PYTHONPATH=/root/mrs_ws/src/informative_planner /usr/bin/python3.12 \\
    build_benchmark_summary.py <dir_do_benchmark>

Lê run_summary.json + gaussian_feeder.log + trajectory.csv de cada rep e o
manifest.json do deck por-rep (se existir) para o número de momentos −Rev.
Reaproveita a paleta escura de build_rrt_gp_deck.py.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.backends.backend_pdf import PdfPages

import build_rrt_gp_deck as deck
from build_rrt_gp_deck import (BG, PANEL, INK, MUT, GRID, GREEN, YELLOW, RED, BOUNDS,
                               new_fig, add_header, style_axes, draw_overlays)

try:
    from pptx import Presentation
    from pptx.util import Inches
    PPTX = True
except Exception:
    PPTX = False

ACCENT = {'dist': '#38bdf8', 'wp': '#4ade80', 'plans': '#ffd34d', 'obs': '#a78bfa'}


def read_gt(path):
    df = pd.read_csv(path)
    return df[['x', 'y']].astype(float)


def gather(rep):
    s = json.loads((rep / 'run_summary.json').read_text())
    log = (rep / 'gaussian_feeder.log').read_text(errors='replace')
    n_obs = sum(1 for _ in (rep / 'gaussian_feeder/observations.csv').open()) - 1
    name = re.search(r'rep\d+', rep.name)
    d = {
        'rep': name.group(0) if name else rep.name,
        'trees': int((s.get('tree_count_final_by_method', {}) or {}).get('tree_mapper', 0)),
        'wp': int(s.get('waypoint_count', 0)),
        'dist': float(s.get('distance_xy_m', 0.0)),
        'dur': float(s.get('run_duration_sec', 0.0)),
        'plans': len(re.findall(r'RRT path ', log)),
        'fresh': len(re.findall(r'RRT path fresh', log)),
        'kept': len(re.findall(r'Mantendo committed', log)),
        'disc': len(re.findall(r'Descartando committed', log)),
        'obs': n_obs,
        'cpu': float((s.get('system_monitor', {}) or {}).get('mean_cpu_total_pct', 0.0)),
        'finish': s.get('finish_reason', ''),
        'gt_path': (s.get('metadata', {}) or {}).get('ground_truth_csv_path'),
    }
    man = rep / 'analysis/rrt_presentation_deck/manifest.json'
    d['rev'] = 0
    if man.exists():
        try:
            d['rev'] = sum(1 for x in json.loads(man.read_text())['moments'] if x['kind'] == 'revisit')
        except Exception:
            pass
    return d


def final_trees(rep):
    fp = rep / 'tree_maps/tree_mapper_map_final.json'
    if not fp.exists():
        return []
    try:
        d = json.loads(fp.read_text())
        return [{'x': float(t['x']), 'y': float(t['y'])} for t in d.get('trees', [])]
    except Exception:
        return []


def ms(vals):
    a = np.asarray(vals, dtype=float)
    return float(a.mean()), float(a.std())


def save(fig, out, name):
    p = out / f'{name}.png'
    fig.savefig(p, dpi=150, facecolor=BG)
    plt.close(fig)
    return p


def hero_tile(ax, x, w, title, big, sub, color=INK):
    ax.add_patch(FancyBboxPatch((x, 0.0), w, 1.0, boxstyle='round,pad=0.006,rounding_size=0.03',
                                transform=ax.transAxes, facecolor=PANEL, edgecolor=GRID, lw=1.0))
    ax.text(x + w / 2, 0.80, title, transform=ax.transAxes, ha='center', va='center', fontsize=12, color=MUT)
    ax.text(x + w / 2, 0.47, big, transform=ax.transAxes, ha='center', va='center', fontsize=26, weight='bold', color=color)
    ax.text(x + w / 2, 0.17, sub, transform=ax.transAxes, ha='center', va='center', fontsize=10.5, color=MUT)


def slide_overview(idx, rows, gt_n, out, label):
    fig = new_fig()
    n = len(rows)
    add_header(fig, 'Benchmark — resumo agregado', label, idx)
    md, sd = ms([r['dist'] for r in rows]); mw, sw = ms([r['wp'] for r in rows])
    mdur, sdur = ms([r['dur'] for r in rows]); full = sum(1 for r in rows if r['trees'] >= gt_n)
    ax = fig.add_axes([0.05, 0.60, 0.90, 0.20]); ax.axis('off')
    tw = 0.235
    hero_tile(ax, 0.00, tw, 'cobertura de árvores', f'{full}/{n}', f'reps com {gt_n}/{gt_n} mapeadas', GREEN)
    hero_tile(ax, 0.255, tw, 'distância XY', f'{md:.0f} m', f'± {sd:.0f} m (dp)')
    hero_tile(ax, 0.510, tw, 'waypoints enviados', f'{mw:.0f}', f'± {sw:.1f} (dp)')
    hero_tile(ax, 0.765, tw, 'duração', f'{mdur:.0f} s', f'± {sdur:.0f} s (dp)')
    # tabela por rep
    axt = fig.add_axes([0.05, 0.08, 0.90, 0.44]); axt.axis('off'); axt.set_xlim(0, 1); axt.set_ylim(0, 1)
    cols = [('rep', 0.03), ('árvores', 0.20), ('waypoints', 0.34), ('dist XY', 0.48),
            ('replans (fresh)', 0.63), ('commits m/d', 0.80), ('−Rev', 0.93)]
    for c, x in cols:
        axt.text(x, 0.95, c, fontsize=11, weight='bold', color=INK, ha='center')
    y = 0.83
    for r in rows:
        axt.add_patch(Rectangle((0.0, y - 0.05), 1.0, 0.088, facecolor=PANEL, edgecolor=GRID, lw=.6))
        tcol = GREEN if r['trees'] >= gt_n else INK
        vals = [(r['rep'], INK), (f"{r['trees']}/{gt_n}", tcol), (str(r['wp']), INK), (f"{r['dist']:.0f} m", INK),
                (f"{r['plans']} ({r['fresh']})", INK), (f"{r['kept']}/{r['disc']}", GREEN if r['disc'] == 0 else RED),
                (str(r['rev']) if r['rev'] else '—', YELLOW if r['rev'] else MUT)]
        for (c, x), (v, col) in zip(cols, vals):
            axt.text(x, y, v, fontsize=11, color=col, ha='center', va='center', weight='bold' if c == 'rep' else 'normal')
        y -= 0.10
    # linha média ± dp
    axt.add_patch(Rectangle((0.0, y - 0.05), 1.0, 0.088, facecolor='#1f2933', edgecolor=GRID, lw=.8))
    mt, st = ms([r['trees'] for r in rows]); mp, sp = ms([r['plans'] for r in rows])
    md2 = [(f'média±dp', INK), (f'{mt:.0f}', INK), (f'{mw:.0f}±{sw:.0f}', INK), (f'{md:.0f}±{sd:.0f}', INK),
           (f'{mp:.0f}±{sp:.0f}', INK), ('—', MUT), (f"{sum(r['rev'] for r in rows)}", MUT)]
    for (c, x), (v, col) in zip(cols, md2):
        axt.text(x, y, v, fontsize=11, color=col, ha='center', va='center', weight='bold')
    fin = rows[0]['finish']
    fig.text(0.05, 0.03, f'{n} reps · término: {fin} · −Rev decisivo em {sum(1 for r in rows if r["rev"])}/{n} reps.',
             fontsize=12, color='#cbd5e1')
    return save(fig, out, '01_overview')


def bar_panel(ax, reps, vals, color, title, unit=''):
    style_axes_bars(ax)
    x = np.arange(len(reps)); m, s = ms(vals)
    ax.axhspan(m - s, m + s, color=color, alpha=0.10, zorder=1)
    ax.axhline(m, color=MUT, lw=1.4, ls=(0, (5, 3)), zorder=2)
    ax.bar(x, vals, width=0.62, color=color, edgecolor=BG, lw=2.0, zorder=3)
    top = max(vals) * 1.18 if max(vals) > 0 else 1
    ax.set_ylim(0, top)
    for xi, v in zip(x, vals):
        ax.text(xi, v + top * 0.02, f'{v:.0f}' if v >= 10 else f'{v:.1f}', ha='center', va='bottom', fontsize=10, color=INK)
    ax.text(len(reps) - 0.5, m, f'  média {m:.0f}{unit}', va='center', ha='left', fontsize=9, color=MUT)
    ax.set_xticks(x); ax.set_xticklabels(reps, fontsize=9, color=MUT)
    ax.set_title(title, color=INK, fontsize=13, weight='bold', pad=8, loc='left')


def style_axes_bars(ax):
    ax.set_facecolor(PANEL)
    ax.grid(True, axis='y', color=GRID, alpha=0.6, lw=0.6)
    ax.tick_params(colors=MUT, labelsize=9)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'):
        ax.spines[sp].set_color(GRID)


def slide_charts(idx, rows, out, label):
    fig = new_fig()
    add_header(fig, 'Métricas por rep (barra) · média ± desvio (faixa)', label, idx)
    reps = [r['rep'].replace('rep', 'r') for r in rows]
    specs = [([r['dist'] for r in rows], ACCENT['dist'], 'distância XY [m]', 'm'),
             ([r['wp'] for r in rows], ACCENT['wp'], 'waypoints enviados', ''),
             ([r['plans'] for r in rows], ACCENT['plans'], 'replanejamentos RRT', ''),
             ([r['obs'] for r in rows], ACCENT['obs'], 'observações no GP', '')]
    positions = [[0.055, 0.50, 0.40, 0.31], [0.545, 0.50, 0.40, 0.31],
                 [0.055, 0.09, 0.40, 0.31], [0.545, 0.09, 0.40, 0.31]]
    for (vals, col, title, unit), pos in zip(specs, positions):
        ax = fig.add_axes(pos)
        bar_panel(ax, reps, vals, col, title, unit)
    fig.text(0.055, 0.025, 'Faixa cinza = média ± 1 desvio-padrão entre reps · linha tracejada = média · rótulo por barra = valor do rep.',
             fontsize=11, color='#cbd5e1')
    return save(fig, out, '02_metricas')


def slide_trajectories(idx, reps_dirs, rows, gt, gt_n, out, label):
    fig = new_fig()
    add_header(fig, 'Trajetórias vs. ground-truth (todos os reps)', label, idx)
    n = len(reps_dirs)
    w = 0.90 / n
    for i, (rep, r) in enumerate(zip(reps_dirs, rows)):
        ax = fig.add_axes([0.05 + i * w, 0.33, w * 0.94, 0.31])
        style_axes(ax)
        traj = pd.read_csv(rep / 'trajectory.csv')
        snap = {'trees': final_trees(rep)}
        draw_overlays(ax, gt, snap, traj, None, legend=False)
        ax.plot(traj['x'], traj['y'], color=INK, lw=0.9, alpha=0.45, zorder=4)
        tcol = GREEN if r['trees'] >= gt_n else INK
        ax.set_title(f"{r['rep']} · {r['trees']}/{gt_n}", color=tcol, fontsize=12, weight='bold', pad=6)
        ax.set_xticklabels([]); ax.set_yticklabels([])
    partial = [r['rep'] for r in rows if r['trees'] < gt_n]
    tail = (f' {", ".join(partial)} deixaram árvore(s) sem mapear (✕ sem círculo).' if partial
            else ' Todos os reps mapearam todas as árvores.')
    fig.text(0.05, 0.20, '✕ ground-truth · ○ árvore mapeada · linha = trajetória voada.' + tail,
             fontsize=12, color='#cbd5e1')
    return save(fig, out, '03_trajetorias')


def main(bench_dir):
    bench = Path(bench_dir).resolve()
    reps_dirs = sorted(d for d in bench.glob('*rep*') if d.is_dir() and (d / 'run_summary.json').exists())
    if not reps_dirs:
        print('nenhum rep com run_summary.json em', bench); return
    rows = [gather(r) for r in reps_dirs]
    gt = read_gt(rows[0]['gt_path']) if rows[0]['gt_path'] and Path(rows[0]['gt_path']).exists() else read_gt(deck.DEFAULT_GT)
    gt_n = len(gt)
    stamp = next((p for p in bench.parts if re.fullmatch(r'\d{2}-\d{2}-\d{2}', p)), bench.name)
    label = f'{bench.parts[-2] if len(bench.parts) >= 2 else ""} · {stamp} · {len(rows)} reps'
    out = bench / 'analysis' / 'benchmark_summary'
    out.mkdir(parents=True, exist_ok=True)

    slides = [slide_overview(1, rows, gt_n, out, label),
              slide_charts(2, rows, out, label),
              slide_trajectories(3, reps_dirs, rows, gt, gt_n, out, label)]

    pdf_path = out / 'benchmark_summary.pdf'
    with PdfPages(pdf_path) as pdf:
        for png in slides:
            fig = new_fig(); ax = fig.add_axes([0, 0, 1, 1]); ax.imshow(plt.imread(png)); ax.axis('off')
            pdf.savefig(fig, dpi=150, facecolor=BG); plt.close(fig)
    pptx_path = None
    if PPTX:
        prs = Presentation(); prs.slide_width = Inches(16); prs.slide_height = Inches(9); blank = prs.slide_layouts[6]
        for png in slides:
            s = prs.slides.add_slide(blank); s.shapes.add_picture(str(png), 0, 0, width=prs.slide_width, height=prs.slide_height)
        pptx_path = out / 'benchmark_summary.pptx'; prs.save(pptx_path)
    (out / 'aggregate_metrics.json').write_text(json.dumps({'benchmark': str(bench), 'reps': rows, 'gt_count': gt_n}, indent=2))
    print('OUT', out); print('slides', len(slides)); print('pdf', pdf_path); print('pptx', pptx_path)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else deck.DEFAULT_RUN)
