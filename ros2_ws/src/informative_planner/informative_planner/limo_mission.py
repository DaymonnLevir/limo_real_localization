#!/usr/bin/env python3
"""Roda uma missão do LIMO e, só depois dela, o pós-processamento.

  ros2 run informative_planner limo_mission map_bounds:="-5,5,-5,5" \
      time_budget_sec:=600 [--no-postprocess] [--no-video]

Argumentos no formato chave:=valor vão direto para limo_ipp.launch.py.
Ctrl-C encerra a missão; os dados salvos ainda são pós-processados.
"""

import os
import signal
import subprocess
import sys
import time

from informative_planner import postprocess_run


def main():
    launch_args = [a for a in sys.argv[1:] if ':=' in a]
    flags = {a for a in sys.argv[1:] if a.startswith('--')}

    given = dict(a.split(':=', 1) for a in launch_args)
    run_name = given.get('run_name') or time.strftime('limo_%Y%m%d_%H%M%S')
    results_root = given.get('results_root') or '~/ipp_results'
    launch_args = [
        a for a in launch_args
        if not a.startswith(('run_name:=', 'results_root:='))
    ] + [f'run_name:={run_name}', f'results_root:={results_root}']
    run_dir = os.path.join(os.path.expanduser(results_root), run_name)

    cmd = ['ros2', 'launch', 'informative_planner', 'limo_ipp.launch.py']
    cmd += launch_args
    print('[limo_mission] ' + ' '.join(cmd), flush=True)
    proc = subprocess.Popen(cmd)
    try:
        proc.wait()
    except KeyboardInterrupt:
        # O terminal já mandou SIGINT ao launch; só espera ele fechar tudo.
        try:
            proc.wait(timeout=60)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            proc.send_signal(signal.SIGTERM)
            proc.wait()

    print(f'[limo_mission] missão encerrada; dados em {run_dir}', flush=True)
    if '--no-postprocess' in flags:
        return
    postprocess_run.run(run_dir, make_video='--no-video' not in flags)
    print(f'[limo_mission] pós-processamento concluído em {run_dir}', flush=True)


if __name__ == '__main__':
    main()
