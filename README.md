# spine multi




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

### Now install the ros componenets

```sh
vcs import src < ./spine-multi/ros/spine_multi_ros/spine_multi.rosinstall
colcon build --symlink-install
source install/setup.sh
```


## Running

### ARL sim

The launch file [sim_launch_all.launch.py](./ros/spine_multi_ros/launch/sim_launch_all.launch.py) will launch
- vision
- nav2 
- some odom publishers

```sh
ros2 launch spine_multi_ros sim_launch_all.launch.py
```


### Now launch SPINE 
```sh
ros2 launch spine_multi_ros spine.launch.py
```