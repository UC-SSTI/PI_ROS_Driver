#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
rospy.init_node("hex_commands")

pub = rospy.Publisher('/target_vel_hex', Float64MultiArray, queue_size=1)
msg = Float64MultiArray()
msg.data = [0, 0, 0, 0, 0, 0, 1]  # Example list of integers

k = 1

def compute_velocity(d, angular_velocity):
    """
    Compute the 6-DOF velocity command to rotate the cube about its center.

    ARGS:
        d (float) - Offset along z-axis between joint origin and cube center.
        angular_velocity (tuple): Desired angular velocities (omega_x, omega_y, omega_z).
    
    Returns 6-DOF velocity command [v_x, v_y, v_z, omega_x, omega_y, omega_z].
    """
    omega_x, omega_y, omega_z = angular_velocity

    v_x = omega_y * d
    v_y = -omega_x * d
    v_z = 0

    result = np.array([v_x, v_y, v_z, omega_x, omega_y, omega_z])
    return result

# 99.12 mm
i = [0]*6
#vel = np.array([0.00001, 0.0001, 0.00005, 0.001, 0.0001, 0.0005])

cube_vel = np.array([0.0, 0.0, 0.0, 0.0003, 0.0001, 0.0001])
period = [3.2, 2.5, 2.75, 10, 8, 3]
rate = 0.5

# Indicates whether the axis has reached the stop position
centered = [True] * 6

while not rospy.is_shutdown():
    
    for v in range(len(cube_vel)):
    
        if centered[v]:
            # Ensures it goes the full range whether than one direction and back to center
            if i[v]*rate > period[v]/2:
                cube_vel[v] = cube_vel[v] * -1
                i[v] = 0
                centered[v] = False
        else:
            if i[v]*rate > period[v]:
                cube_vel[v] = cube_vel[v] * -1
                i[v] = 0

    vel = compute_velocity(-0.09912, cube_vel[3:])
    print(vel)



    msg.data = cmd = [vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], 10000.0]

    for ii in range(len(i)):
        i[ii] += 1
    pub.publish(msg)
    rospy.sleep(rate)