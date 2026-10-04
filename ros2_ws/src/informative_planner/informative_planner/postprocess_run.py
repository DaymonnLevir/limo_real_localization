#!/usr/bin/env python3
"""Pós-processamento de uma run do LIMO (roda DEPOIS da missão).

Tudo que é pesado e não é necessário para a decisão online fica aqui:
vídeo de propagação do GP (replay do observations.csv). Os dados brutos já
estão em disco quando isto roda.

  ros2 run informative_planner postprocess_run ~/ipp_results/limo_<stamp>
"""

import argparse
import csv
import json
import os
import pathlib
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory


def _read_bounds(run_dir):
    summary = run_dir / 'run_summary.json'
    if summary.exists():
        with open(summary, encoding='utf-8') as fh:
            bounds = json.load(fh).get('metadata', {}).get('map_bounds')
        if bounds and len(bounds) == 4:
            return [float(v) for v in bounds]
    env = os.environ.get('IPP_MAP_BOUNDS')
    if env:
        return [float(v) for v in env.split(',')]
    return None


def _confirmed_trees_csv(run_dir, out_path):
    """Extract confirmed tree_mapper trees (x,y) for the video overlay."""
    src = run_dir / 'tree_mapper' / 'tree_map_final.csv'
    if not src.exists():
        return None
    rows = []
    with open(src, newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            confirmed = str(row.get('confirmed', 'true')).strip().lower()
            if confirmed in ('1', 'true', 'yes'):
                rows.append((row['x'], row['y']))
    with open(out_path, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.writer(fh)
        writer.writerow(['x', 'y'])
        writer.writerows(rows)
    return out_path


def run(run_dir, make_video=True):
    run_dir = pathlib.Path(run_dir).expanduser().resolve()
    log = []
    observations = run_dir / 'gaussian_feeder' / 'observations.csv'

    if make_video:
        bounds = _read_bounds(run_dir)
        if not observations.exists():
            log.append(f'video: sem {observations}; pulando.')
        elif bounds is None:
            log.append('video: map_bounds desconhecido; pulando.')
        else:
            script = pathlib.Path(
                get_package_share_directory('informative_planner')
            ) / 'scripts' / 'gp_propagation_video.py'
            cmd = [
                sys.executable, str(script),
                '--observations', str(observations),
                '--bounds', *[str(v) for v in bounds],
                '--out', str(run_dir / 'gp_propagation.mp4'),
            ]
            trees = _confirmed_trees_csv(
                run_dir, run_dir / 'postprocess_trees_confirmed.csv'
            )
            if trees is not None:
                cmd += ['--trees-csv', str(trees)]
            print('[postprocess] ' + ' '.join(cmd), flush=True)
            result = subprocess.run(cmd, check=False)
            log.append(f'video: exit={result.returncode}')

    with open(run_dir / 'postprocess_log.txt', 'a', encoding='utf-8') as fh:
        for line in log:
            fh.write(line + '\n')
    for line in log:
        print('[postprocess] ' + line, flush=True)
    return run_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    run(args.run_dir, make_video=not args.no_video)


if __name__ == '__main__':
    main()
