#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from chai_msgs.msg import ObjectCmd
from rift import PyRift


class OculusDriver(object):
    def __init__(self):
        rospy.init_node('openhmd_ros')
        self.pub = rospy.Publisher('/openhmd/pose', PoseStamped)
        self.pub2 = rospy.Publisher('/chai/env/ecm/Command', ObjectCmd)
        self.hmd = None

    def start(self):
        self.hmd = PyRift()
        print self.hmd.getDeviceInfo()
        if self.hmd:
            self.run()

    def run(self):
        # r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            d = self.hmd.poll()
            p = self.hmd.poll2()
            if len(d) != 4:
            	continue
            if len(p) != 3:
            	continue

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "/map"
            pose_stamped.header.stamp=rospy.Time.now()
            pose_stamped.pose.orientation = Quaternion(d[0], d[1], d[2], d[3])
            # pose_stamped.pose.orientation = Quaternion(0.044,0,0,0.999)

            # euler = euler_from_quaternion(d)
            # pose.position = Point(euler[0], euler[1], euler[2])
            pose_stamped.pose.position = Point(p[0], p[1], p[2])

            self.pub.publish(pose_stamped)

            obj_stamped = ObjectCmd()
            obj_stamped.header.frame_id = "/map"
            obj_stamped.header.stamp=rospy.Time.now()
            obj_stamped.pose.orientation = Quaternion(d[0], d[1], d[2], d[3])
            # obj_stamped.pose.orientation = Quaternion(0,0,0,1);

            # euler = euler_from_quaternion(d)
            # pose.position = Point(euler[0], euler[1], euler[2])
            obj_stamped.pose.position = Point(p[0], p[1], p[2])
            obj_stamped.pos_ctrl = True

            self.pub2.publish(obj_stamped)

            # r.sleep()

if __name__ == "__main__":
    driver = OculusDriver()
    driver.start()
