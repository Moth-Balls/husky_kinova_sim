# husky_kinova_sim
1. Clone repo 

```
mkdir -p ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/Moth-Balls/husky_kinova_sim.git
```

2. Install dependecies

This should install all the dependencies but I may have missed one.

```
sudo apt-get install ros-humble-gz-ros2-control ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-moveit
```

Sometimes rosdep works but it may still miss some.


```
rosdep update

rosdep install --from-paths src --ignore-src -y
```

1. Building

```
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

1. Launching 

You can launch an empty world with this command.

```
ros2 launch husky_kinova_sim husky_kinova_empty.launch.py
```

Headless world with green objects can be launched as well.

```
ros2 launch husky_kinova_sim husky_kinova_headless.launch.py
```
