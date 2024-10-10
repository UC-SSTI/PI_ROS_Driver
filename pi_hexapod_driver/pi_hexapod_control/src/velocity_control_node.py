#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class HexTrajectoryControl(object):
    def __init__(self):
        rospy.init_node("hex_trajectory_control")

        self.vel_pub = rospy.Publisher(
                        '/hex_vel_controller/command', 
                        Float64MultiArray,
                        queue_size=10)

        self.pos_pub = rospy.Publisher(
                        '/joint_group_pos_controller/command',
                        Float64MultiArray,
                        queue_size=10)

    def publish(self, position: list, velocity: list):
        # TODO: Currently the first element in the velocity list
        # is set directly as the hexapod system velocity.
        pos_msg = Float64MultiArray()
        pos_msg.data = position
        
        vel_msg = Float64MultiArray()
        vel_msg.data = velocity

        self.pos_pub.publish(pos_msg)
        self.vel_pub.publish(vel_msg)


    def calculate_magnitude(self, v):
        return np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

    def calculate_direction(self, v):
        magnitude = self.calculate_magnitude(v)
        direction = [
            v[0] / magnitude,
            v[1] / magnitude,
            v[2] / magnitude
        ]
        return direction

    def is_vectors_equal(self, v1, v2):
        if len(v1) != len(v2):
            return False

        for i in range(len(v1)):
            # Use isclose since we're comparing floats
            if not math.isclose(v1[i], v2[i]):
                return False

        return True


    def execute_trajectory(self, trajectory):
        for point in trajectory:
            # Calculate average speed
            # Note this doesn't account for angular speed.
            lin_speed = self.calculate_magnitude(point['vel'][:3])
            

            # Check velocity is in the same direction as position
            lin_pos_dir = self.calculate_direction(point['pos'][:3])
            lin_vel_dir = self.calculate_direction(point['vel'][:3])
            ang_pos_dir = self.calculate_direction(point['pos'][3:])
            ang_vel_dir = self.calculate_direction(point['vel'][3:])

            if not self.is_vectors_equal(lin_pos_dir, lin_vel_dir):
                raise Exception("Linear Position and Velocity Directions Differ")

            if not self.is_vectors_equal(ang_pos_dir, ang_vel_dir):
                raise Exception("Angular Position and Velocity Directions Differ")

            # Calculate time to reach point
            # Does not account for acceleration (make delta_v between points small)
            distance = self.calculate_magnitude(point['pos'])
            #time_for_point = 

    def run(self):
        trajectory = [
            {"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             "vel": [20.0, 10.0, 10.0, 10.0, 10.0, 10.0]},

            {"pos": [0.0, 0.016, -0.0065, -0.174535, 0.0, 0.0],
             "vel": [20.0, 10.0, 10.0, 10.0, 10.0, 10.0]},

            {"pos": [0.0, 0.0, 0.0065, 0.0, 0.0, 0.0],
             "vel": [20.0, 10.0, 10.0, 10.0, 10.0, 10.0]},

            {"pos": [0.0, 0.0, -0.0065, 0.174535, 0.0, 0.0],
             "vel": [20.0, 10.0, 10.0, 10.0, 10.0, 10.0]},
        ]


        for point in trajectory:
            self.publish(point["pos"], point["vel"])
            rospy.sleep(3)

if __name__ == "__main__":
    node = HexTrajectoryControl()
    node.run()