#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def callback(data, pub):
    # print(data.name)
    try:
        index = data.name.index('go1_gazebo')  # Replace with your robot's name
        pose = data.pose[index]
        # rospy.loginfo("\nRobot Pose: {}".format(pose))
        pub.publish(pose)
    except ValueError:
        pass  # Robot not found in the current message

def listener():
    rospy.init_node('robot_pose_listener', anonymous=True)
    
    # Create a publisher for the 'localization_pose' topic
    pub = rospy.Publisher('/odom_gazebo', Pose, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback, pub)
    rospy.spin()

if __name__ == '__main__':
    listener()
