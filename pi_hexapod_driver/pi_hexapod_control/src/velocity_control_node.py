#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import time
import warnings

class HexTrajectoryControl(object):
    def __init__(self):
        rospy.init_node("hex_trajectory_control")

        self.qdot_pub = rospy.Publisher(
                        '/pi_hardware_interface/qdot_cmd',
                        JointTrajectory,
                        queue_size=10
        )

        # Each qdot trajectory point will execute for this time (in ms).

    def publish(self, trajectory: list, qdot_period:float=50):
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
        """
        trajectory_msg = JointTrajectory()

        for point in trajectory:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.velocities = point
            print(point)
            for element in point:
                if abs(element) > 20:
                    print(f"WARNING: Velocity command ({element}) is > max (20 mm/s).")
            trajectory_msg.points.append(trajectory_point)

        trajectory_msg.points[0].effort.append(qdot_period)

        self.qdot_pub.publish(trajectory_msg)


    def run(self):
        trajectory = [
           [0.0, 0.0, -50, 0.0, 0.0, 0.0] for i in range(1)
        ]

        #while not rospy.is_shutdown():
        for i in range(2):
            self.publish(trajectory, 500)
            rospy.sleep(1)

        

if __name__ == "__main__":
    node = HexTrajectoryControl()
    node.run()