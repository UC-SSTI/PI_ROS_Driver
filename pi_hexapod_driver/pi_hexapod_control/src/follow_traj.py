#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as sciR
import matplotlib.pyplot as plt

def quat_msg_to_np(q):
    return np.array([q.x, q.y, q.z, q.w])

class TrajFollower():
    def __init__(self):
        rospy.init_node("traj_follower")

        self.pub = rospy.Publisher('/target_vel_hex', Float64MultiArray, queue_size=1)
        

        rospy.Subscriber('top', Pose, self.pose_callback)
        rospy.Subscriber('tf', TFMessage, self.tf_callback)

        self.goal_q = None
        self.platform_q = None
        self.rate = 0.5

        self.k = 0.01

        self.time_0 = None
        self.time = None
        self.time_list = []
        self.goal_list = []
        self.curr_list = []

    def pose_callback(self, msg):
        self.goal_q = quat_msg_to_np(msg.orientation)

    def tf_callback(self, msg):
        u_q = None
        v_q = None
        w_q = None
        for t in msg.transforms:
            if t.child_frame_id == "u_link":
                u_q = quat_msg_to_np(t.transform.rotation)

                # Time in seconds
                time = t.header.stamp.secs + t.header.stamp.nsecs / 1e9
                if self.time_0 is None:
                    self.time_0 = time
                self.time = time - self.time_0

            elif t.child_frame_id == "v_link":
                v_q = quat_msg_to_np(t.transform.rotation)
            elif t.child_frame_id == "w_link":
                w_q = quat_msg_to_np(t.transform.rotation)

        if u_q is None or v_q is None or w_q is None:
            #print("u:", u_q, "v:", v_q, "w:", w_q)
            return

        platform_rot = sciR.from_quat(u_q) * sciR.from_quat(v_q) * sciR.from_quat(w_q)
        self.platform_q = platform_rot.as_quat()

    def publish(self, ang_vel):
        msg = Float64MultiArray()
        msg.data = [0., 0., 0., ang_vel[0], ang_vel[1], ang_vel[2], 10000.0]
        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.goal_q is None or self.platform_q is None:
                print("goal:", self.goal_q, "platform:", self.platform_q)
                continue

            diff = sciR.from_quat(self.goal_q) * sciR.from_quat(self.platform_q).inv()
            diff = diff.as_euler('xyz')
            print("diff:", diff)

            self.publish(self.k * diff)

            self.goal_list.append(sciR.from_quat(self.goal_q).as_euler('xyz', degrees=True))
            self.curr_list.append(sciR.from_quat(self.platform_q).as_euler('xyz', degrees=True))
            self.time_list.append(self.time)
            
            rospy.sleep(self.rate)

    def plot(self):
        diff = np.array(self.goal_list) - np.array(self.curr_list)
        plt.plot(self.time_list, diff[:, 0], label="Roll Error")
        plt.plot(self.time_list, diff[:, 1], label="Pitch Error")
        plt.plot(self.time_list, diff[:, 2], label="Yaw Error")

        # plt.plot(self.time_list, np.array(self.curr_list)[:, 0], label="Roll Cmd")
        # plt.plot(self.time_list, np.array(self.curr_list)[:, 1], label="Pitch Cmd")
        # plt.plot(self.time_list, np.array(self.curr_list)[:, 2], label="Yaw Cmd")

        plt.xlabel("Time (s)")
        plt.ylabel("Rotation (deg)")
        plt.title("Satellite Target Tracking Error")
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    follower = TrajFollower()
    follower.run()

    follower.plot()
