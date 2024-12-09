#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import time
import warnings
from sensor_msgs.msg import JointState
import sys



class HexTrajectoryControl(object):
    def __init__(self):
        rospy.init_node("hex_trajectory_control")
        self.trajectory_list = [
           [0.0, 0.0, 0, 0, 0.0, 0.0] for i in range(1)
        ]
        self.how_long = 0.0
        self.joint_states_vector = [0,0,0,0,0,0]
        self.get_vel_cmd = rospy.Subscriber('/target_vel_hex', Float64MultiArray ,self.get_vel_cmd_callback)
        
        self.joint_feedback = rospy.Subscriber('/joint_states', JointState ,self.joint_feedback_callback)

        self.qdot_pub = rospy.Publisher(
                        '/pi_hardware_interface/qdot_cmd',
                        JointTrajectory,
                        queue_size=1
        )

        self.cb_count = 0

        # Each qdot trajectory point will execute for this time (in ms).

    def print_count(self):
        print("CMD:", self.cb_count)
        if self.cb_count < 1000:
            self.cb_count += 1
        else:
            self.cb_count = 0

    def get_vel_cmd_callback(self, msg):
        self.trajectory_list = [
           msg.data[:6] for i in range(1)
        ]
        # print(msg.data)
        self.how_long = msg.data[-1]*1000
        self.publish(self.trajectory_list, self.how_long)
        self.print_count()
    
    def joint_feedback_callback(self, msg):
        target_names = ['cart_x', 'cart_y', 'cart_z', 'ang_u', 'ang_v', 'ang_w']
        # Extract indices of target joints
        indices = [msg.name.index(name) for name in target_names if name in msg.name]

        # Extract positions of target joints using indices
        self.joint_states_vector = [i * 1000 for i in [msg.position[i] for i in indices]]
        # print(self.joint_states_vector)
        
    def is_within_range(self, values, min_range, max_range):
        """
        Check if each value in the list is within the specified range.

        Args:
            values (list): List of numbers to check.
            min_range (list): List of minimum values for each component.
            max_range (list): List of maximum values for each component.

        Returns:
            bool: True if all values are within range, False otherwise.
        """
        # Ensure all lists have the same length
        if len(values) != len(min_range) or len(values) != len(max_range):
            raise ValueError("All lists must have the same length.")

        # Check if each value is within the range
        return all(min_val <= val <= max_val for val, min_val, max_val in zip(values, min_range, max_range))

    def publish(self, trajectory: list, qdot_period:float=50, zero=False):
        """
        Commands hexapod to execute trajectory.

        PARAMS:
          trajectory - A list of list where the inner list defines the 
                       velocity for each axis (X, Y, Z, U, V, W).
                       Velocity is in mm/s for translation and mrad/s for rotation.

                       The outer list defines a point on the trajectory.
                       Each point is executed after the qdot_period.

          qdot_period - The amount of time (in ms) to execute each point.
                        The hexapod will move at the commanded rate for this long.
          zero - Doesn't perform command and goes to zero positions.
        """
        trajectory_msg = JointTrajectory()

        if not zero:
            for point in trajectory:
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.velocities = point
                print(point)

                if self.is_within_range(self.joint_states_vector, [-50, -50, -25, -260, -260, -520], [50, 50, 25, 260, 260, 520]):
                    print("All joints are within range.")
                else:
                    print("Some joints are out of range.")
                    for num in self.joint_states_vector:
                        print(f"{num:.4f}")

                for element in point:
                    if abs(element) > 20:
                        print(f"WARNING: Velocity command ({element}) is > max (20 mm/s).")
                trajectory_msg.points.append(trajectory_point)

            trajectory_msg.points[0].effort.append(qdot_period)
        else:
            print("Zeroing...")
            trajectory_msg.points.append(JointTrajectoryPoint())
            trajectory_msg.points[0].effort.append(qdot_period)
            trajectory_msg.points[0].effort.append(1)

        self.qdot_pub.publish(trajectory_msg)


    def run(self):
        trajectory = [
           [0, 0.0, 20.0, 0, 0.0, 0] for i in range(1)
        ]
        
        for i in range(2):
            self.publish(trajectory,500)
            rospy.sleep(1)
        
    

if __name__ == "__main__":
    node = HexTrajectoryControl()
    # node.run()

    if len(sys.argv) > 1:
        while not rospy.is_shutdown():
            node.publish([0.0]*6, qdot_period=1, zero=True)

    while not rospy.is_shutdown():
        rospy.spin()