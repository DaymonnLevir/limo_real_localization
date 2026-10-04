# Planner Benchmark Environment

This benchmark setup lets you replace the informative GP planner with simpler
planner strategies while keeping the rest of the system unchanged:

- Gazebo forest world from `octomap_gazebo_forest`
- MRS Octomap mapping and Octomap Planner
- `pointcloud_to_laserscan`
- `tree_measuring/tree_node`
- `tree_measuring/tree_map_plotter`
- the same `/uav1/octomap_planner/goto` interface used by
  `gaussian_feeder`

So yes, the intended pipeline makes sense:

1. Start the simulation with `octomap_gazebo_forest`.
2. Start the perception chain that publishes `/uav1/tree_list` and
   `/uav1/point_quality`.
3. Run one planner node.
4. The planner sends waypoints to MRS.
5. MRS uses its OctoMap to generate safe trajectories and avoid obstacles.

Do not run `gaussian_feeder` at the same time as an active benchmark strategy
such as `random_waypoint` or `lawnmower`, because both nodes would send goals
to `/uav1/octomap_planner/goto`. The exception is `strategy:=informative`,
where `benchmark_planner` is passive and only records the run.

## Implemented Planners

The benchmark entry point is:

```bash
ros2 run informative_planner benchmark_planner
```

It supports these strategies:

| Strategy | Parameter value | Behavior |
|---|---|---|
| Random Waypoint | `random_waypoint` | Chooses a random candidate waypoint inside `d_step` of the current UAV position, waits for MRS to finish, then chooses again. |
| Lawnmower | `lawnmower` | Builds a boustrophedon grid over `map_bounds` and sends each waypoint in order. |
| Informative GP planner | `informative` | Passive benchmark mode. The benchmark records the run while `gaussian_feeder` sends waypoints. |

Both strategies use `DroneController`, the same wrapper used by
`gaussian_feeder`, so a new waypoint is sent only after:

- Octomap Planner diagnostics have been received;
- the current goal is finished and the planner is idle;
- the settle time and minimum command interval have passed.

## Build

From the workspace root:

```bash
cd ~/mrs_ws
colcon build --packages-select informative_planner
source install/setup.bash
```

If you also changed `tree_measuring` or `pointcloud_to_laserscan`, build the
full workspace instead.

## One Benchmark Run

Start the simulation in the first terminal:

```bash
cd ~/mrs_ws/src/octomap_gazebo_forest
./start.sh
```

In batch mode, the active world is selected by the third argument and resolved
through `octomap_gazebo_forest/config/benchmark_worlds/<world>.conf`. That file
is the shared source for the SDF, spawn, planner bounds, lawnmower start, and
ground truth.

Start the LaserScan slice:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/uav1/ouster/points \
  -r scan:=/scan_1_3m \
  -p use_sim_time:=true \
  -p target_frame:=uav1/gps_baro_origin \
  -p min_height:=1.25 \
  -p max_height:=1.35 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p range_min:=0.0 \
  -p range_max:=80.0 \
  -p use_inf:=true
```

Start tree perception and the plotter:

```bash
ros2 launch tree_measuring tree_measuring.launch.py \
  enable_map_plotter:=true
```

Before each benchmark repetition, either restart `tree_measuring` or clear its
accumulated map:

```bash
ros2 service call /uav1/reset_tree_map std_srvs/srv/Trigger "{}"
```

Run Random Waypoint:

```bash
ros2 run informative_planner benchmark_planner --ros-args \
  -p use_sim_time:=true \
  -p strategy:=random_waypoint \
  -p run_name:=world01_random_rep01 \
  -p time_budget_sec:=100.0 \
  -p random_seed:=1
```

Run Lawnmower:

```bash
ros2 run informative_planner benchmark_planner --ros-args \
  -p use_sim_time:=true \
  -p strategy:=lawnmower \
  -p run_name:=world01_lawnmower_rep01 \
  -p time_budget_sec:=100.0
```

Run the original informative planner under the same benchmark logger:

Terminal 1, start the passive benchmark recorder:

```bash
ros2 run informative_planner benchmark_planner --ros-args \
  -p use_sim_time:=true \
  -p strategy:=informative \
  -p run_name:=world01_informative_rep01 \
  -p time_budget_sec:=100.0
```

Terminal 2, start the planner that actually sends the goals:

```bash
ros2 run informative_planner gaussian_feeder --ros-args \
  -p use_sim_time:=true \
  -p map_bounds:="[-16.0, 16.0, -16.0, 16.0]"
```

In `strategy:=informative`, `benchmark_planner` does not send goals to
`/<uav_name>/octomap_planner/goto`. It only records odometry, `/tree_list`,
and waypoint commands published by the shared `DroneController` on
`/<uav_name>/planner_waypoint_cmd`.

## Important Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `strategy` | `random_waypoint` | `random_waypoint`, `lawnmower`, or `informative`. |
| `time_budget_sec` | `config.TIME_BUDGET` | Main experiment budget. The planner stops and saves artifacts when this time is reached. |
| `map_bounds` | `config.MAP_BOUNDS` | `[x_min, x_max, y_min, y_max]` area used by both strategies. |
| `world_name` | empty | Scenario name recorded in benchmark provenance. |
| `world_file` | empty | SDF path recorded in benchmark provenance. |
| `ground_truth_csv_path` | empty | Ground-truth path recorded in benchmark provenance. |
| `flight_height` | `config.FLIGHT_HEIGHT` | Waypoint altitude sent to MRS. |
| `d_step` | `config.MAX_WAYPOINT_STEP` | Random Waypoint maximum jump length. Also used as the default lawnmower spacing. |
| `min_dist` | `1.5` | Minimum Random Waypoint distance from the current UAV position. |
| `waypoint_grid_resolution` | `config.WAYPOINT_GRID_RESOLUTION` | Candidate grid spacing for Random Waypoint. |
| `random_seed` | `-1` | Negative means random seed. Use a non-negative integer for repeatable runs. |
| `lawnmower_line_spacing` | `d_step` | Distance between lawnmower rows. |
| `lawnmower_track_spacing` | `d_step` | Distance between points along each row. |
| `lawnmower_margin` | `0.0` | Optional margin removed from `map_bounds`. |
| `external_waypoint_topic` | `/<uav_name>/planner_waypoint_cmd` | Waypoint event topic recorded by passive `informative` mode. |
| `tree_list_topic` | `/<uav_name>/tree_list` | Legacy `tree_measuring` map source. |
| `tree_measuring_map_frame` | `uav1/gps_baro_origin` | Configured frame for the headerless legacy map. |
| `tree_mapper_map_topic` | `/tree_map_full` | Confirmed `tree_mapper` map consumed by the benchmark. |
| `output_dir` | `/tmp/mrs_forest_benchmarks` | Root directory for benchmark artifacts. |
| `run_name` | auto timestamp | Subdirectory name for this run. |

## Saved Artifacts

Each run creates:

```text
/tmp/mrs_forest_benchmarks/<run_name>/
  run_summary.json
  trajectory_raw.csv
  trajectory.csv
  waypoints.csv
  tree_map_final.json
  tree_snapshots.jsonl
  trajectory_tree_map.png
```

`trajectory_raw.csv` stores every odometry sample received by the benchmark.
`trajectory.csv` stores a lighter sampled version. The PNG trajectory uses
`trajectory_raw.csv` so the visual path is not shortened by sample filtering.
`run_summary.json` stores the total traveled distance:

- `distance_xy_m`
- `distance_3d_m`

`tree_map_final.json` stores the final map published by `tree_node` through
`/uav1/tree_list`. This is the machine-readable tree map for metric scripts.
When the benchmark is started by `octomap_gazebo_forest/run_benchmark_batch.sh`,
it also consumes the confirmed `tree_mapper` map from `/tree_map_full`. Both
mapping methods are saved separately under `tree_maps/`; use
`tree_maps/manifest.json` to identify the mapping method, benchmark strategy,
topic, frame, and artifact paths. The root `tree_map_final.json` remains the
legacy `tree_measuring` map for backward compatibility. See
`octomap_gazebo_forest/TREE_MAPPER_BENCHMARK_INTEGRATION.md` for the complete
layout and lifecycle.
`trajectory_tree_map.png` stores a visual map of the trajectory executed during
the benchmark, the waypoints sent to MRS, and the trees discovered by
`tree_node`.
The visual tree map PNG is still produced by `tree_map_plotter`; by default it
is saved as:

```text
/tmp/tree_measuring_tree_map.png
```

## Metrics

Use the same time budget for every planner and scenario.

### Number of identified trees

Use `tree_count_final` from `run_summary.json`, or count the entries in
`tree_map_final.json`.

### Total distance traveled

Use `distance_xy_m` from `run_summary.json` for horizontal navigation cost.
Use `distance_3d_m` if you want altitude changes included.

### DAP RMSE

The benchmark logger saves the estimated `dbh` values from `tree_node`, but
RMSE also needs ground-truth DAP/DBH for each tree in each Gazebo world.
Create one ground-truth JSON per world, for example:

```json
[
  {"id": "tree_001", "x": -15.6, "y": -17.2, "dbh": 0.90},
  {"id": "tree_002", "x": -20.4, "y": -12.6, "dbh": 0.90}
]
```

For worlds that use `tree_simple_thick`, the model trunk radius is `0.45 m`,
so the DBH/DAP ground truth is `0.90 m`. If your three new worlds use models
with different trunk radii or scales, store the true `dbh` in the JSON.

A metric script should match each detected tree to the nearest unmatched
ground-truth tree within a fixed radius, such as `1.5 m`, then compute:

```text
RMSE_DAP = sqrt(mean((dbh_estimated - dbh_ground_truth)^2))
```

Keep the matching radius fixed for all planners and all worlds.

## Repetition Protocol

The paper protocol should be:

1. Select one world in `octomap_gazebo_forest/session.yml`.
2. Start Gazebo/MRS.
3. Start `pointcloud_to_laserscan`.
4. Start `tree_measuring`.
5. Run exactly one planner with the chosen `time_budget_sec`.
6. Save the benchmark output directory and the tree map PNG.
7. Restart the simulation before the next repetition, or at minimum call
   `/uav1/reset_tree_map` and make sure the OctoMap/planner state is clean.

Run each planner ten times per world. Use explicit run names:

```text
world01_random_rep01
world01_random_rep02
world01_lawnmower_rep01
world01_informative_rep01
world02_random_rep01
...
```

For Random Waypoint, use fixed seeds (`1`, `2`, ..., `10`) when you want
repeatability. For Lawnmower, repetitions should be identical unless the
simulation or perception stack is stochastic. For the informative planner,
restart Gazebo and `tree_measuring` before each repetition so the GP, tree map,
OctoMap, and UAV start from the same clean state.

## Extending With More Planners

Add new active strategies in:

```text
informative_planner/benchmark_strategies.py
```

Each strategy only needs:

- `next_waypoint(current_xy)`
- optional `on_waypoint_sent(waypoint)`
- optional `is_done()`

The ROS/MRS connection, time budget, odometry logging, waypoint logging, and
tree-list logging remain in `benchmark_planner.py`.

For planners that already run as their own ROS node, use the passive
`informative`/`external` mode instead of adding another active strategy. The
external planner should use `DroneController` so its waypoint commands are
published to `/<uav_name>/planner_waypoint_cmd` and saved in `waypoints.csv`.
