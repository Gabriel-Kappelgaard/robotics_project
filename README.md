UR5e Motion Planning with ROS 2 Jazzy and MoveIt 2

A ROS 2 project for experimenting with motion planning and Cartesian robot movements using a Universal Robots UR5e.

The project uses MoveIt 2 for motion planning and pymoveit2 as the Python interface. The robot runs using the UR ROS 2 driver's mock hardware interface.

No physical robot or Gazebo installation is required.

1. Development environment
Component	Version / Configuration
Operating system	Ubuntu 24.04 LTS
ROS 2	Jazzy
Robot	Universal Robots UR5e
Robot driver	Universal Robots ROS 2 Driver
Hardware interface	ros2_control mock hardware
Motion planning	MoveIt 2
Python interface	pymoveit2
Robot configuration	ur_moveit_config
Visualization	RViz2
Build system	colcon / CMake
ROS package	ur_app_bringup
Simulation approach

This project uses the UR driver's mock hardware interface:

bash
use_mock_hardware:=true


The mock hardware interface allows the ROS 2 control system to simulate robot joint states and trajectory execution.

It provides a development environment for testing:

ROS 2 communication.
Joint state publishing.
MoveIt motion planning.
Trajectory execution.
Python-based robot control.

It is important to distinguish this from Gazebo simulation. Mock hardware does not simulate physical dynamics, gravity, contact forces or interactions with a simulated environment.

2. Repository structure

The repository is organized as a ROS 2 workspace:

text
robotics_project/
│
├── README.md
│
└── src/
    │
    └── ur_app_bringup/
        ├── CMakeLists.txt
        ├── package.xml
        │
        ├── launch/
        │   └── simple_move.launch.py
        │
        ├── scripts/
        │   └── simple_move.py
        │
        └── config/
            └── ...


The ur_app_bringup package contains the custom application and launch configuration.

The exact contents of the config directory depend on the local project configuration.

3. Install ROS 2 Jazzy

The development environment uses Ubuntu 24.04 LTS.

Install ROS 2 Jazzy Desktop using the official instructions:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Source ROS 2:

bash
source /opt/ros/jazzy/setup.bash


To source ROS 2 automatically in new terminals:

bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

4. Install dependencies

Update the package lists:

bash
sudo apt update


Install the UR driver, MoveIt and development dependencies:

bash
sudo apt install \
    ros-jazzy-ur \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit \
    ros-jazzy-pymoveit2 \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    python3-colcon-common-extensions \
    python3-rosdep


Initialize rosdep if it has not already been initialized:

bash
sudo rosdep init
rosdep update


Install package dependencies:

bash
cd ~/robotics_project

rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y

5. Clone the repository

Create the workspace:

bash
mkdir -p ~/robotics_project/src
cd ~/robotics_project


Clone the repository:

bash
git clone https://github.com/Gabriel-Kappelgaard/robotics_project.git .


The repository should now be available at:

text
~/robotics_project


Check the package structure:

bash
find src/ur_app_bringup -maxdepth 3 -type f

6. Build the workspace

The custom package uses ament_cmake.

Build the workspace:

bash
cd ~/robotics_project

colcon build --packages-select ur_app_bringup


Source the workspace:

bash
source install/setup.bash


For convenience, add workspace sourcing to .bashrc:

bash
echo "source ~/robotics_project/install/setup.bash" >> ~/.bashrc


Each new terminal should source both ROS 2 and the workspace overlay.

Rebuild after changing source files
bash
cd ~/robotics_project

colcon build --packages-select ur_app_bringup

source install/setup.bash


For a clean rebuild:

bash
rm -rf build/ur_app_bringup install/ur_app_bringup

colcon build --packages-select ur_app_bringup

CMake installation requirements

Since ur_app_bringup is an ament_cmake package, its CMakeLists.txt must install its launch files and Python executables.

For example, if the source files are organized as shown above, the following installation rules can be added before ament_package():

cmake
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/simple_move.py
  DESTINATION lib/${PROJECT_NAME}
)


The Python script must also have executable permissions:

bash
chmod +x src/ur_app_bringup/scripts/simple_move.py


The package manifest should declare its runtime dependencies, including the relevant ROS 2, MoveIt and pymoveit2 dependencies.

7. Start the UR5e mock hardware

Open Terminal 1.

Source ROS 2:

bash
source /opt/ros/jazzy/setup.bash


Start the UR5e driver:

bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    use_mock_hardware:=true \
    launch_rviz:=false


This starts the UR driver with mock hardware.

The driver provides:

Robot description.
Joint state publishing.
ROS 2 control.
Joint trajectory controllers.

No physical robot connection is required.

8. Start MoveIt 2

Open Terminal 2.

Source ROS 2:

bash
source /opt/ros/jazzy/setup.bash


Start the standard UR MoveIt configuration:

bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e


This starts the MoveIt planning system.

The main components include:

move_group
Planning scene monitor.
Robot state monitoring.
Motion planning pipeline.
Trajectory execution interface.
RViz2 visualization.

MoveIt communicates with the running UR driver and its controllers.

If RViz is already running, check the available launch arguments:

bash
ros2 launch ur_moveit_config ur_moveit.launch.py --show-args


Disable RViz in the launch configuration if supported.

9. Verify the simulation

Before starting the Python application, verify that the robot state and controllers are available.

Check ROS 2 nodes
bash
ros2 node list

Check joint states
bash
ros2 topic echo /joint_states


You should see six joint positions corresponding to the UR5e joints.

Check controllers
bash
ros2 control list_controllers


Verify that:

The joint state broadcaster is active.
The trajectory controller configured for your UR setup is active.

The exact controller name should be taken from this command rather than assumed.

Check robot description
bash
ros2 topic list


Verify that /robot_description is available.

10. Run the Python application

Open Terminal 3.

Source ROS 2 and the workspace:

bash
source /opt/ros/jazzy/setup.bash
source ~/robotics_project/install/setup.bash


Run the application:

bash
ros2 run ur_app_bringup simple_move


Alternatively, if the custom launch file is installed:

bash
ros2 launch ur_app_bringup simple_move.launch.py


The custom launch file is intended to start the existing UR MoveIt configuration and the Python application.

MoveIt must be available before the Python application sends its first motion request.

11. Motion planning using pymoveit2

The application uses pymoveit2 to communicate with the existing MoveIt move_group.

Unlike moveit_py, pymoveit2 does not create a separate MoveIt planning instance. It communicates with the running MoveIt system using ROS 2 interfaces.

Robot configuration

The planning group is:

python
GROUP = "ur_manipulator"


The six UR5e joints are:

python
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


The reference frame and end-effector are:

python
BASE_LINK = "base_link"
END_EFFECTOR = "tool0"

Cartesian motion

The example application moves the robot to a predefined Cartesian pose.

Target position:

python
position = [0.4, 0.2, 0.4]


Target orientation:

python
orientation = [1.0, 0.0, 0.0, 0.0]


The position is specified in metres.

The orientation is a quaternion in XYZW order.

The motion request is sent using:

python
self.moveit2.move_to_pose(
    position=position,
    quat_xyzw=orientation,
    frame_id=BASE_LINK,
    target_link=END_EFFECTOR,
)


The application waits for trajectory execution:

python
success = self.moveit2.wait_until_executed()

12. Troubleshooting
No joint states received

Example error:

text
Joint states are not available yet — waiting...
Cannot plan because no joint states were received within 10.0 s.


Possible causes:

The UR driver is not running.
Mock hardware was not enabled.
The joint state broadcaster is not active.
The Python node started before the driver was ready.

Check:

bash
ros2 topic echo /joint_states


Check the controllers:

bash
ros2 control list_controllers


Make sure the UR driver was started with:

bash
use_mock_hardware:=true

MoveIt cannot execute trajectories

Check:

bash
ros2 control list_controllers


Verify that the trajectory controller configured for the UR setup is active.

Also verify that MoveIt has a matching controller configuration.

Multiple RViz windows

The UR MoveIt launch configuration can start RViz automatically.

If you already have an RViz instance running, disable RViz in the additional launch configuration where supported.

Python module not found

Check that the package is installed:

bash
ros2 pkg list | grep pymoveit2


Install it if necessary:

bash
sudo apt install ros-jazzy-pymoveit2

Package or executable not found

Rebuild the workspace:

bash
cd ~/robotics_project

colcon build --packages-select ur_app_bringup

source install/setup.bash


Check the available executables:

bash
ros2 pkg executables ur_app_bringup


If simple_move is missing, check the executable installation rules in CMakeLists.txt.

13. Recommended development workflow

The normal development setup uses three terminals.

Terminal	Purpose
1	UR5e mock hardware
2	MoveIt 2
3	Custom Python application

Start the components in this order:

Start the UR driver with mock hardware.
Start MoveIt 2.
Verify joint states and controllers.
Run the pymoveit2 application.

When modifying the Python application or launch files, rebuild the package and source the workspace again.

14. Future development

Possible extensions include:

Joint-space motion commands.
Relative joint movements.
Cartesian path planning.
Configurable velocity and acceleration scaling.
Pilz industrial motion planners (PTP, LIN and CIRC).
Custom motion planning pipelines.
Integration with Gazebo for physics-based simulation.
Connection to a physical UR5e robot.

These are potential future extensions and are not required by the current mock hardware setup.

15. References
ROS 2 Jazzy documentation
Universal Robots ROS 2 Driver
MoveIt 2 documentation
pymoveit2
UR MoveIt configuration
Project repository

Project: UR5e Motion Planning Demo
Operating system: Ubuntu 24.04 LTS
ROS distribution: Jazzy
Robot: UR5e
Simulation: UR ros2_control mock hardware
Motion planning: MoveIt 2
Python interface: pymoveit2
Gazebo: Not used
