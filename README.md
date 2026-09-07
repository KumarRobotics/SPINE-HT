# spine multi

SPINE-multi coordinates a team of heterogeneous robots (ground and aerial) on missions
described in natural language. The pipeline works in three stages:

1. **Decomposition** (`src/spine_multi/decomp`) — an LLM breaks a mission down into a
   sequence of tasks drawn from a fixed navigation/inspection/mapping API
   (e.g. `ugv_navigate`, `ugv_inspect`, `ugv_map_region`, `ugv_explore_to_coord`).
2. **Allocation** (`src/spine_multi/allocation`) — tasks are assigned to robots by
   solving an LP over per-platform capabilities (e.g. Jackal, Husky, UAV).
3. **Execution** (`src/spine_multi/spine`) — each robot runs a SPINE loop that queries
   an LLM against its live semantic graph to pick the next grounded action
   (`explore_region`, `map_region`, `inspect`, `clarify`, `goto`, `answer`,
   `extend_map`, `replan`), validating and retrying on invalid LLM output.

`src/spine_multi/baselines` contains an alternate LLM-coordinator implementation used
for comparison against SPINE.

## Installing

### First install the python package in your ros ws
```sh
cd ~/ws/src
git clone https://github.com/ZacRavichandran/spine-multi && cd spine-multi
python -m pip install -e .
cd ..
```

Additional packages required
- [GroundingDino](https://github.com/ZacRavichandran/GroundingDino)

### Now install the ros components

```sh
vcs import src < ./spine-multi/ros/spine_multi_ros/spine_multi_https.rosinstall
colcon build --symlink-install
source install/setup.sh
```

(use `spine_multi_git.rosinstall` instead if you prefer cloning over SSH)

## Running

### Coordinator

`multi_spine.launch.py` is the top-level entry point, typically run on the basestation.
It brings up the `spine_multi_node`, which reads a per-robot config
(`robot_config`, a `spine_multi.yaml`) and optionally starts the `mocha` basestation
comms stack (`use_mocha`).

```sh
ros2 launch spine_multi_ros multi_spine.launch.py robot_config:=<PATH_TO_CONFIG> use_mocha:=True
```

Each robot then runs its own SPINE node against an initial semantic graph:

```sh
ros2 launch spine_multi_ros spine.launch.py namespace:=<ROBOT_NS> init_graph:=<PATH_TO_GRAPH>
```

### Platforms

Jackal, Husky, and Spot all launch through the same pattern: a namespaced group that
brings up static transforms, navigation, vision, and a platform-specific autonomy
server, then optionally recording and `mocha`.

```sh
ros2 launch spine_multi_ros jackal_launch_all.launch.py
ros2 launch spine_multi_ros husky_launch_all.launch.py
ros2 launch spine_multi_ros spot_launch_all.launch.py
```

What differs per platform:

| | Jackal | Husky | Spot |
|---|---|---|---|
| Static transforms | `jackal_static_transforms.launch.py` | `jackal_static_transforms.launch.py` | `spot_static_transforms.launch.py` |
| Navigation | nav2 (`nav2_jackal_stvox.yaml`) | nav2 (`nav2_husky.yaml`) | none |
| Vision | `vision.launch.py` | `vision_jackal.launch.py` (off by default, `use_vision`) | `vision_jackal.launch.py` (always on) |
| Autonomy server | `jackal_autonomy_server.py` | `jackal_autonomy_server.py` | `spot_autonomy_server.py` |
| `use_mocha` default | `false` | `true` | `false` |
| Recording | `record_jackal.launch.py` | `record_husky.launch.py` | `record_spot.launch.py` |

### ARL sim

The launch file [sim_launch_all.launch.py](./ros/spine_multi_ros/launch/sim_launch_all.launch.py) will launch
- vision
- nav2
- some odom publishers

```sh
ros2 launch spine_multi_ros sim_launch_all.launch.py
```


### (optional) Rviz
```sh
cd <SPINE-MULTI-DIR>
rviz2 -d ./ros/spine_multi_ros/rviz/sim.rviz
```
