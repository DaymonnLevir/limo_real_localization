"""LIMO + Nav2: tree_mapper v2 (árvores + arbustos) + planejador informativo.

Visibilidade (point_score) com a MESMA semântica das runs do artigo: o
point_quality_node conta as árvores aceitas pelo critério geométrico no frame
de nuvem atual (/tree_mapper/internal/detections_full do classificador v2) e
publica /<robot>/point_quality; o feeder usa isso no GP e /tree_map_full como
obstáculos no RRT. tree_measuring não é usado.

Pressupõe que o Nav2 (com /navigate_to_pose ativo e TF map->odom->base_link)
e os drivers do robô (nuvem 3D em cloud_topic) já estão rodando.

Tudo de uma run vai para <results_root>/<run_name>/:
  run_summary.json, trajectory*.csv, waypoints.csv, point_quality.csv,
  tree_maps/, system_monitor.csv            (benchmark_planner, passivo)
  gaussian_feeder/  observations.csv, gp_grid.csv, gp_map_*.png
  tree_mapper/      tree_map_final.csv/json, histórico, detecções
  tree_mapper_params.yaml                   (parâmetros efetivos do tree-mapper)
  bag/              rosbag (se record_bag:=true)

Fim de missão: o benchmark_planner encerra no time_budget_sec, publica
/ipp/mission_done (o feeder cancela o goal do Nav2 e salva) e, após
shutdown_grace_sec, o launch inteiro é encerrado. Vídeos/figuras pesadas:
`ros2 run informative_planner postprocess_run <run_dir>` — o `limo_mission`
faz isso automaticamente.

Exemplo:
  ros2 launch informative_planner limo_ipp.launch.py \
      map_bounds:="-5.0,5.0,-5.0,5.0" time_budget_sec:=600
"""

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# Tópicos leves gravados no bag (a nuvem 3D só com bag_pointcloud:=true).
BAG_TOPICS = [
    '/tf', '/tf_static', '/odom', '/scan', '/cmd_vel', '/map', '/amcl_pose',
    '/plan', '/navigate_to_pose/_action/status',
    '/tree_map_full', '/shrub_map_full',
    '/tree_mapper/internal/detections_full', '/ipp/mission_done',
]


def _as_bool(text):
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _write_tree_mapper_yaml(context, run_dir, overrides):
    """Copy the tree_mapper config yaml with the launch overrides."""
    cfg = context.launch_configurations['tree_mapper_config']
    src = cfg if os.path.isabs(cfg) else os.path.join(
        FindPackageShare('tree_mapper').perform(context), 'config', cfg,
    )
    if not os.path.isfile(src):
        raise RuntimeError(f'tree_mapper_config não encontrado: {src}')
    with open(src, encoding='utf-8') as fh:
        params = yaml.safe_load(fh)
    for node_name, values in overrides.items():
        params.setdefault(node_name, {}).setdefault('ros__parameters', {})
        params[node_name]['ros__parameters'].update(values)
    dst = os.path.join(run_dir, 'tree_mapper_params.yaml')
    with open(dst, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(params, fh, sort_keys=False)
    return dst


def _setup(context):
    def arg(name):
        return context.launch_configurations[name]

    robot_name = arg('robot_name')
    planning_frame = arg('planning_frame')
    odom_topic = arg('odom_topic')
    cloud_topic = arg('cloud_topic')
    results_root = os.path.abspath(os.path.expanduser(arg('results_root')))
    run_name = arg('run_name')
    run_dir = os.path.join(results_root, run_name)
    os.makedirs(run_dir, exist_ok=True)

    fuser_overrides = {
        'observer_pose_topic': odom_topic,
        'observer_target_frame': planning_frame,
        'default_frame_id': planning_frame,
    }
    tree_mapper_yaml = _write_tree_mapper_yaml(context, run_dir, {
        'vegetation_candidate_detector': {
            'input_cloud_topic': cloud_topic,
            'target_frame': planning_frame,
            'self_filter_frame': arg('base_frame'),
            # pontos da banda fora da area de teste nao viram candidatos
            'map_bounds': arg('map_bounds'),
        },
        'tree_map_fuser': fuser_overrides,
        'shrub_map_fuser': fuser_overrides,
    })
    tree_stack_launch = os.path.join(
        FindPackageShare('tree_mapper').perform(context),
        'launch', 'tree_stack_interface.launch.py',
    )
    bounds = [v.strip() for v in arg('map_bounds').split(',')]
    if len(bounds) != 4:
        raise RuntimeError(f'map_bounds inválido: {arg("map_bounds")!r} (esperado x_min,x_max,y_min,y_max)')

    actions = [
        LogInfo(msg=f'[limo_ipp] run_dir = {run_dir}'),
        # config.py do informative_planner lê estes valores no import.
        SetEnvironmentVariable('IPP_PLANNING_FRAME', planning_frame),
        SetEnvironmentVariable('IPP_MAP_BOUNDS', arg('map_bounds')),
        SetEnvironmentVariable('IPP_CONTROLLER_BACKEND', 'nav2'),

        # Plots ao vivo do tree_mapper só com show_tree_plots:=true (CPU durante a missão).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tree_stack_launch),
            launch_arguments={
                'config_yaml': tree_mapper_yaml,
                'run_fuser': 'true',
                'run_plotter': arg('show_tree_plots'),
                'run_cluster_plotter': arg('show_tree_plots'),
                # plots com eixos fixos exatamente nos map_bounds (mesma área do GP)
                'fixed_axes': 'true',
                'x_min': bounds[0], 'x_max': bounds[1],
                'y_min': bounds[2], 'y_max': bounds[3],
                'axis_margin': '0.0',
                'use_boundary_bounds': 'false',
                'run_snapshot_exporter': 'false',
                'run_output_dir': os.path.join(run_dir, 'tree_mapper'),
                'run_id': run_name,
            }.items(),
        ),

        # Zona segura do Nav2: OccupancyGrid livre dentro de map_bounds (+margem) e letal fora,
        # consumido pela static_layer dos costmaps (nav2_no_map.yaml, map_topic /bounds_map).
        Node(
            package='informative_planner',
            executable='bounds_map_publisher',
            output='screen',
            parameters=[{
                'map_bounds': arg('map_bounds'),
                'frame_id': planning_frame,
                'margin': float(arg('bounds_margin')),
                'use_sim_time': _as_bool(arg('use_sim_time')),
            }],
        ),

        Node(
            package='informative_planner',
            executable='point_quality_node',
            output='screen',
            parameters=[{
                'uav_name': robot_name,
                'odom_topic': odom_topic,
                'planning_frame': planning_frame,
                'use_sim_time': _as_bool(arg('use_sim_time')),
            }],
        ),

        Node(
            package='informative_planner',
            executable='gaussian_feeder',
            # Sem name=: ele renomearia também o nav2_controller, que roda
            # no mesmo processo (__node remapeia todos os nós).
            output='screen',
            parameters=[{
                'uav_name': robot_name,
                'controller_backend': 'nav2',
                'navigate_action': arg('navigate_action'),
                'odom_topic': odom_topic,
                'show_plot': _as_bool(arg('show_plot')),
                'require_tree_map': True,
                'output_dir': run_dir,
                'use_sim_time': _as_bool(arg('use_sim_time')),
            }],
        ),
    ]

    # Gravador passivo: não manda goals, só registra e controla o budget.
    benchmark = Node(
        package='informative_planner',
        executable='benchmark_planner',
        output='screen',
        parameters=[{
            'uav_name': robot_name,
            'strategy': 'gaussian_feeder',
            'odom_topic': odom_topic,
            'output_dir': results_root,
            'run_name': run_name,
            'time_budget_sec': float(arg('time_budget_sec')),
            'tree_measuring_map_frame': planning_frame,
            'world_name': 'limo_real',
            'use_sim_time': _as_bool(arg('use_sim_time')),
        }],
    )
    actions.append(benchmark)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=benchmark,
        on_exit=[
            LogInfo(msg=(
                '[limo_ipp] benchmark_planner encerrou; desligando em '
                f'{arg("shutdown_grace_sec")} s (feeder/tree_mapper salvando).'
            )),
            TimerAction(
                period=float(arg('shutdown_grace_sec')),
                actions=[EmitEvent(event=Shutdown(reason='mission complete'))],
            ),
        ],
    )))

    if _as_bool(arg('record_bag')):
        topics = list(BAG_TOPICS)
        if odom_topic not in topics:
            topics.append(odom_topic)
        topics += [
            f'/{robot_name}/point_quality',
            f'/{robot_name}/planner_waypoint_cmd',
        ]
        if _as_bool(arg('bag_pointcloud')):
            topics.append(cloud_topic)
        actions.append(ExecuteProcess(
            # nice: o bag nunca disputa CPU com a percepção/planejamento.
            cmd=['nice', '-n', '10', 'ros2', 'bag', 'record',
                 '-o', os.path.join(run_dir, 'bag')] + topics,
            output='log',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='limo'),
        DeclareLaunchArgument('planning_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/depth/points',
            description='Nuvem 3D de entrada do vegetation_candidate_detector',
        ),
        DeclareLaunchArgument(
            'navigate_action', default_value='/navigate_to_pose'),
        DeclareLaunchArgument(
            'map_bounds',
            default_value='-5.0,5.0,-5.0,5.0', #OS MAP BOUNDS SÃO REFERENTES AO INICIO DO FAST-LIO? 
            description='x_min,x_max,y_min,y_max da área de teste [m]',
        ),
        DeclareLaunchArgument(
            'time_budget_sec', default_value='600.0',
            description='Duração da missão [s], contada da 1a odometria',
        ),
        DeclareLaunchArgument(
            'results_root', default_value='~/ipp_results'),
        DeclareLaunchArgument(
            'run_name',
            default_value=time.strftime('limo_%Y%m%d_%H%M%S'),
        ),
        DeclareLaunchArgument(
            'shutdown_grace_sec', default_value='10.0',
            description='Espera após o fim do budget antes de encerrar tudo',
        ),
        DeclareLaunchArgument(
            'tree_mapper_config', default_value='interface_topics.yaml',
            description='YAML do tree_mapper (nome em tree_mapper/config ou caminho absoluto); '
                        'ex.: interface_topics_cilindros.yaml para cilindros artificiais',
        ),
        DeclareLaunchArgument(
            'bounds_margin', default_value='0.6',
            description='Folga [m] além de map_bounds que o Nav2 ainda considera livre (/bounds_map)',
        ),
        DeclareLaunchArgument(
            'show_tree_plots', default_value='false',
            description='Plots ao vivo do tree_mapper (mapa e clusters; custam CPU durante a missão)',
        ),
        DeclareLaunchArgument(
            'show_plot', default_value='false',
            description='Plot ao vivo do GP (custa CPU durante a missão)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument(
            'bag_pointcloud', default_value='false',
            description='Grava também a nuvem 3D (pesado em disco)',
        ),
        OpaqueFunction(function=_setup),
    ])
