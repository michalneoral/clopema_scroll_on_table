#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, WrenchStamped
from tf2_msgs.msg import TFMessage
from clopema_robot import ClopemaRobotCommander
import tf

def publisher(data, arg):
    pub=arg[0];
    robot=arg[1];
    pos=arg[2];
    try:
        t = data.header.stamp
        time_stamp=listener.getLatestCommonTime('/ctu_floor', '/r1_ee')
        listener.waitForTransform('/ctu_floor', '/r1_ee',time_stamp,rospy.Duration(.001))
        (trans,rot) = listener.lookupTransform('/ctu_floor', '/r1_ee', time_stamp)
        print rospy.Time.now()
        pos.pose.position.x=trans[0]
        pos.pose.position.y=trans[1]
        pos.pose.position.z=trans[2]
        pos.pose.orientation.x=rot[0]
        pos.pose.orientation.y=rot[1]
        pos.pose.orientation.z=rot[2]
        pos.pose.orientation.w=rot[3]
        pos.header.stamp = t
        pub.publish(pos)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Error!"

def retransf(listener):
    pub = rospy.Publisher('r1ee', PoseStamped)
    print "published"
    pos = PoseStamped()    
    pos.header.frame_id = "/r1ee"
    rospy.Subscriber("/r1_force_data_filtered", WrenchStamped, publisher, [pub, listener, pos], queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('r1ee_pub4force', anonymous=True)
    listener = tf.TransformListener()

    retransf(listener)
