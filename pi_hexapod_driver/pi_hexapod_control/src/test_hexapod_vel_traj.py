#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import matplotlib.pyplot as plt
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

cube_vel = np.array([0.0, 0.0, 0.0, 0.0003, 0.0001, 0.0002])
period = [3.2, 2.5, 2.75, 10, 8, 3]
rate = 0.5

# Indicates whether the axis has reached the stop position
centered = [True] * 6

velocities = []

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

    velocities.append(vel)

    msg.data = cmd = [vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], 10000.0]

    for ii in range(len(i)):
        i[ii] += 1
    pub.publish(msg)
    rospy.sleep(rate)


time = np.linspace(0, rate*len(velocities), len(velocities))

plt.plot(time, 1000 * np.array(velocities)[:,3], label="X Axis Angular Velocity")
plt.plot(time, 1000 * np.array(velocities)[:,4], label="Y Axis Angular Velocity")
plt.plot(time, 1000 * np.array(velocities)[:,5], label="Z Axis Angular Velocity")


plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (deg/s)")
plt.title("Satellite Angular Velocity Over Time")
plt.legend()
plt.grid(True)
plt.show()

plt.clf()

position_x = 1000 * np.cumsum(np.array(velocities)[:,3]) * rate
position_y = 1000 * np.cumsum(np.array(velocities)[:,4]) * rate
position_z = 1000 * np.cumsum(np.array(velocities)[:,5]) * rate


plt.plot(time, position_x, label="X Axis Rotation")
plt.plot(time, position_y, label="Y Axis Rotation")
plt.plot(time, position_z, label="Z Axis Rotation")

plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Satellite Attitude Over Time")
plt.legend()
plt.grid(True)
plt.show()