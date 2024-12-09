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

    return [v_x, v_y, v_z, omega_x, omega_y, omega_z]

i = [0]*6
vel = np.array([0.00001, 0.0001, 0.00005, 0.00025, 0.0001, 0.0005])
period = [3.2, 2.5, 2.75, 1, 2, 3]
rate = 0.5

while not rospy.is_shutdown():
    
    for v in range(len(vel)):
    
        if i[v]*rate > period[v]:
            vel[v] = vel[v] * -1
            i[v] = 0

    print(vel)

    msg.data = cmd = [vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], 10000.0]

    for ii in range(len(i)):
        i[ii] += 1
    pub.publish(msg)
    rospy.sleep(rate)