#!/usr/bin/env python3

import rospy
import subprocess
import threading
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class OdomMonitor:
    def __init__(self):
        self.odom_received = False
        self.last_time_received = rospy.Time.now()
        self.timeout = rospy.Duration(10)  # 10 seconds timeout

        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback)
        self.check_thread = threading.Thread(target=self.check_odometry)
        self.check_thread.start()

    def odom_callback(self, msg):
        self.odom_received = True
        self.last_time_received = rospy.Time.now()

    def check_odometry(self):
        rate = rospy.Rate(1)  # check once per second
        while not rospy.is_shutdown():
            if self.odom_received:
                # Reset the flag and continue
                self.odom_received = False
                rospy.loginfo("Odometry received")
                continue

            if rospy.Time.now() - self.last_time_received > self.timeout:
                rospy.logwarn("Odometry not received for 10 seconds, Killing rtabmap node...")
                self.restart_rtabmap_node()
                self.last_time_received = rospy.Time.now()  # Reset the time

            rate.sleep()

    @staticmethod
    def restart_rtabmap_node():
        # Commands to restart your rtabmap node, depends on your setup
        subprocess.call(["rosnode", "kill", "/rtabmap/cam_odometry"])  # Kill the node
        subprocess.call(["rosnode", "kill", "/rtabmap/rtabmap"])  # Kill the node
        # Wait for the node to go down
        rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('diagnostic_node', anonymous=True)
    monitor = OdomMonitor()
    rospy.spin()
