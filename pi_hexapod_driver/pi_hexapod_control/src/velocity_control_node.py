#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray

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
        pos_msg = Float64MultiArray()
        pos_msg.data = position
        
        vel_msg = Float64MultiArray()
        vel_msg.data = velocity

        self.pos_pub.publish(pos_msg)
        self.vel_pub.publish(vel_msg)

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