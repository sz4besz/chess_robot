# onrobot_rg_control ROS2 HUMBLE

The `onrobot_rg_control` package is designed to control the RG2-FT gripper using ROS2 HUMBLE and Modbus TCP.
(For Foxy change to Foxy branch)

## Installation

To use this package, you first need to clone it into your ROS 2 workspace. Use the following command:

```bash
cd ~/ros2_ws/src
git clone https://github.com/fraunhoferhhi/Ros2-OnRobot-RG2-FT.git
```
# Installing Dependencies
Dependencies and requirements for this package can be installed from the package.xml or requirements.txt files. Execute the following commands to install the necessary dependencies:
# Install dependencies from package.xml
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
# Install Python pymodbus
```
pip install pymodbus
```
# Build the packages
```
cd ~/ros2_ws/
```
```
colcon build
```
```
source install/setup.bash
```
# Establishing Connection and Starting Control
To establish a connection to the RG2-FT gripper, first start the bringup launch file with the following command:
```
ros2 launch onrobot_rg_control bringup.launch.py
```
In a separate terminal, you can then start the control node to control the gripper:
```
ros2 run onrobot_rg_control onrobot_rg_simple_controller
```

## Programmatic Python control

Start the bringup node first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch onrobot_rg_control bringup.launch.py ip:=10.150.0.2 gripper:=rg2ft
```

Then control the gripper from another Python process on the same machine:

```python
from onrobot_rg_control.remote_gripper import RemoteGripper, init, shutdown

init()
gripper = RemoteGripper(gripper='rg2ft')
try:
    if not gripper.wait_for_connection():
        raise RuntimeError('The gripper TCP node is not running')

    gripper.move(width=500, force=200)
    status = gripper.get_status()
    if status is None:
        raise RuntimeError('No gripper status received')
    print(f'width={status.grip_width}, busy={status.busy}')
finally:
    gripper.destroy_node()
    shutdown()
```

Widths are in 0.1 mm units and forces are in 0.1 N units. The Python process and bringup process must use the same ROS 2 domain.
With these steps, you have completed the basic setup and are now ready to control the RG2-FT gripper using ROS and Modbus TCP.
