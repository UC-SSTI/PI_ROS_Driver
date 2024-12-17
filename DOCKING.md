# Docking

The following provides instructions to execute the docking experiment.

### Satellite Hexapod Motion

Terminal 1
1. `source ~/catkin_ws/devel/setup.bash`
2. `roslaunch pi_hexapod_control interactive_marker.launch`

Terminal 2
1. `source ~/catkin_ws/devel/setup.bash`
2. `cd ~/catkin_ws/src/PI_ROS_Driver/pi_hexapod_driver/pi_hexapod_control/src`
3. `python3 velocity_control_node.py`

Terminal 3
1. `source ~/catkin_ws/devel/setup.bash`
2. `cd ~/catkin_ws/src/PI_ROS_Driver/pi_hexapod_driver/pi_hexapod_control/src`
3. `python3 test_hexapod_vel_traj.py`

### Docker Hexapod

Terminal 4
1. `export ROS_MASTER_URI=http://localhost:1234`
2. `roscore -p 1234`

Terminal 5
1. `export ROS_MASTER_URI=http://localhost:1234`
2. `source ~/catkin_ws/devel/setup.bash`
3. `roslaunch pi_hexapod_control interactive_marker.launch`

Terminal 6
1. `export ROS_MASTER_URI=http://localhost:1234`
2. `source ~/catkin_ws/devel/setup.bash`
3. `cd ~/catkin_ws/src/PI_ROS_Driver/pi_hexapod_driver/pi_hexapod_control/src`
4. `python3 velocity_control_node.py`

Terminal 7
1. `export ROS_MASTER_URI=http://localhost:1234`
2. `source ~/catkin_ws/devel/setup.bash`
3. `cd ~/catkin_ws/src/PI_ROS_Driver/pi_hexapod_driver/pi_hexapod_control/src`
4. `python3 follow_traj.py`

## Stopping
To stop executing, `ctrl+c` with `test_hexapod_vel_traj.py` stops sending commands and plots the trajectory, `ctrl+c` with `follow_traj.py` stops sending commands and plots the error.

To zero run `python3 velocity_control_node.py 1`

