# UR5e Motion Planning (ROS 2 Jazzy + MoveIt 2)

Move a Universal Robots UR5e through a list of positions using MoveIt 2 and the
[pymoveit2](https://github.com/AndrejOrsula/pymoveit2) Python interface.

The robot runs on the UR driver's **mock hardware**, so you don't need a real robot
or Gazebo. Mock hardware simulates joint states and trajectory execution, but not
physics (no gravity, contact or collisions with the world).

| | |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Robot | UR5e (mock hardware) |
| Planning | MoveIt 2 via pymoveit2 |
| Package | `ur_app_demo` (Python) |

---

## Quick start

Do the one-time [setup](#setup) first. Then open three terminals:

**Terminal 1: robot driver (mock hardware)**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e use_mock_hardware:=true launch_rviz:=false
```

**Terminal 2: MoveIt (opens RViz)**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```

**Terminal 3: move through the waypoints**
```bash
source /opt/ros/jazzy/setup.bash
source ~/robotics_project/install/setup.bash
ros2 run ur_app_demo waypoints
```

The robot in RViz should visit each waypoint in order.

---

## Editing the waypoints

Open [`src/ur_app_control/ur_app_demo_py/ur_app_demo/waypoints.py`](src/ur_app_control/ur_app_demo_py/ur_app_demo/waypoints.py)
and edit the list at the top:

```python
WAYPOINTS = [
    {"name": "home",    "joints": [0, -90, 90, -90, -90, 0]},
    {"name": "point_A", "pose": [0.40,  0.20, 0.40, 180, 0, 0]},
    {"name": "point_B", "pose": [0.40, -0.20, 0.40, 180, 0, 0], "linear": True},
    {"name": "home",    "joints": [0, -90, 90, -90, -90, 0]},
]
```

Each waypoint uses **one** of two formats:

| Key | Format | Units |
|---|---|---|
| `"pose"` | `[x, y, z, roll, pitch, yaw]` of `tool0`, relative to `base_link` | metres, degrees |
| `"joints"` | `[base, shoulder, elbow, wrist1, wrist2, wrist3]` | degrees |

Add `"linear": True` to a pose waypoint to move in a straight line instead of a
free path. `"name"` is only used in the log output.

Settings below the list:

| Setting | Meaning |
|---|---|
| `SPEED`, `ACCELERATION` | 0.0 to 1.0, fraction of the robot's maximum |
| `PAUSE_SECONDS` | Pause at each waypoint |

If a waypoint can't be reached, the robot stops there and the rest is skipped.

After editing, rebuild:
```bash
cd ~/robotics_project
colcon build --packages-select ur_app_demo
source install/setup.bash
```

> Tip: `roll=180, pitch=0, yaw=0` points the tool straight down.

---

## Setup

### 1. Install ROS 2 Jazzy

Follow the [official install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 2. Install dependencies
```bash
sudo apt update
sudo apt install \
    ros-jazzy-ur \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit \
    ros-jazzy-pymoveit2 \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    python3-colcon-common-extensions \
    python3-rosdep
```
If `ros-jazzy-pymoveit2` isn't found, clone
[pymoveit2](https://github.com/AndrejOrsula/pymoveit2) into `src/` and build it
with the workspace.

### 3. Clone and build
```bash
git clone https://github.com/Gabriel-Kappelgaard/robotics_project.git ~/robotics_project
cd ~/robotics_project

sudo rosdep init   # only the first time on this machine
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select ur_app_demo
source install/setup.bash
```

Optional, to source the workspace in every new terminal:
```bash
echo "source ~/robotics_project/install/setup.bash" >> ~/.bashrc
```

---

## Repository layout

```text
robotics_project/
└── src/
    ├── ur_app_bringup/                 # (empty CMake package, for later bringup files)
    └── ur_app_control/
        └── ur_app_demo_py/             # ROS package "ur_app_demo"
            ├── setup.py                # registers the executables below
            ├── package.xml
            ├── launch/
            │   └── simple_move.launch.py
            └── ur_app_demo/
                ├── simple_move.py      # move to one pose
                └── waypoints.py        # move through a list of poses (edit here)
```

To add another script, put it in `ur_app_demo/` with a `main()` function and add a
line to `console_scripts` in `setup.py`.

---

## Robot configuration

These values are used by the script and match the standard `ur_moveit_config`:

| | |
|---|---|
| Planning group | `ur_manipulator` |
| Base frame | `base_link` |
| End effector | `tool0` |
| Joints | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |

---

## Checking that everything runs

```bash
ros2 node list                 # driver and move_group should be listed
ros2 topic echo /joint_states  # six joint positions
ros2 control list_controllers  # joint_state_broadcaster + a trajectory controller "active"
```

## Troubleshooting

| Problem | Fix |
|---|---|
| "Joint states are not available yet" | Start Terminal 1 first and check `use_mock_hardware:=true`. Check `ros2 topic echo /joint_states`. |
| Plans but doesn't move | Run `ros2 control list_controllers` and check the trajectory controller is active. |
| A waypoint fails | The pose may be out of reach or in collision. Try a point closer to the base or change the orientation. |
| `No executable found` | Rebuild and `source install/setup.bash`. Check `ros2 pkg executables ur_app_demo`. |
| `No module named pymoveit2` | Install it (see Setup step 2). |
| Two RViz windows | Keep `launch_rviz:=false` in Terminal 1. |

---

## Ideas for later

- Pilz industrial planners (PTP / LIN / CIRC)
- Gazebo for physics simulation
- Running on a real UR5e (`robot_ip:=...` and `use_mock_hardware:=false`)

## References

- [ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/)
- [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt 2 docs](https://moveit.picknik.ai/main/index.html)
- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2)
