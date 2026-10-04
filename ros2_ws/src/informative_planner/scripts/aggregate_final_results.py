#!/usr/bin/python3.12
"""
Agrega as métricas dos benchmarks finais (RESULTADOS_ARTIGO) por mundo/método.
Métricas por run (dos dados brutos):
  - DBH RMSE [cm]  : sqrt(mean(diameter_sq_error_m2)) sobre árvores casadas
  - trees found    : nº de detecções casadas com o ground truth (recall = /GT)
  - distance [m]   : distance_xy_m do run_summary
Saída: tabela legível + linhas LaTeX (mean +/- std).
Reexecute quando o lawnmower terminar para atualizar a tabela do pôster.
"""
import csv, glob, json, math, os, sys
import numpy as np

DEST = "/root/mrs_ws/RESULTADOS_ARTIGO"
GT = {"forest_bands_10min": 39, "forest_clustered_10min": 39, "forest_diverse_10min": 39}
WORLDS = ["forest_bands_10min", "forest_clustered_10min", "forest_diverse_10min"]
METHODS = ["proposed", "myopic", "lawnmower"]


def f(x):
    try: return float(x)
    except Exception: return None


def run_metrics(rep):
    tm = glob.glob(os.path.join(rep, "tree_maps", "**", "tree_map_final.csv"), recursive=True)
    rs = os.path.join(rep, "run_summary.json")
    if not tm or not os.path.exists(rs):
        return None
    rows = list(csv.DictReader(open(tm[0])))
    matched = [r for r in rows if (f(r.get("gt_diameter_m")) or 0) > 0]
    sq = [f(r["diameter_sq_error_m2"]) for r in matched if f(r.get("diameter_sq_error_m2")) is not None]
    dbh_rmse = math.sqrt(sum(sq) / len(sq)) if sq else float("nan")
    dist = json.load(open(rs)).get("distance_xy_m", float("nan"))
    return {"dbh_rmse_cm": dbh_rmse * 100, "trees": len(matched), "dist": dist}


def agg(vals):
    a = np.asarray([v for v in vals if v is not None and np.isfinite(v)], float)
    return (float(a.mean()), float(a.std())) if len(a) else (float("nan"), float("nan"))


print(f"{'world':22s} {'method':10s} {'n':>2s}  {'DBH RMSE [cm]':>16s}  {'trees/GT':>10s}  {'dist [m]':>14s}")
print("-" * 84)
latex = []
for w in WORLDS:
    for m in METHODS:
        reps = sorted(glob.glob(os.path.join(DEST, w, m, "rep*")))
        mets = [run_metrics(r) for r in reps]
        mets = [x for x in mets if x]
        if not mets:
            print(f"{w:22s} {m:10s} {0:>2d}  {'-- (pendente) --':>16s}")
            latex.append((w, m, None))
            continue
        rm, rs = agg([x["dbh_rmse_cm"] for x in mets])
        tm, ts = agg([x["trees"] for x in mets])
        dm, ds = agg([x["dist"] for x in mets])
        gt = GT[w]
        print(f"{w:22s} {m:10s} {len(mets):>2d}  {rm:6.2f} +/- {rs:4.2f}    "
              f"{tm:4.1f}/{gt:<2d}       {dm:6.1f} +/- {ds:4.1f}")
        latex.append((w, m, dict(rm=rm, rs=rs, tm=tm, ts=ts, gt=gt, dm=dm, ds=ds)))

print("\n% ---- linhas LaTeX (DBH RMSE | trees | distance) ----")
for w, m, d in latex:
    lbl = {"proposed": "Proposed (RRT)", "myopic": "Myopic", "lawnmower": "Lawnmower"}[m]
    if d is None:
        print(f"{w.split('_')[1]:10s} & {lbl:15s} & -- & -- & -- \\\\  % pendente")
    else:
        print(f"{w.split('_')[1]:10s} & {lbl:15s} & "
              f"${d['rm']:.2f}\\pm{d['rs']:.2f}$ & "
              f"${d['tm']:.1f}/{d['gt']}$ & "
              f"${d['dm']:.0f}\\pm{d['ds']:.0f}$ \\\\")
