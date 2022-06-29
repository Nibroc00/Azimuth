#!/user/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

def callback(t):
    rospy.loginfo(t.transform.translation.x)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("vicon/SOMR/SOMR", geometry_msgs.msg.TransformStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
