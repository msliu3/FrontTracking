#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   ros_marker.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/11/28 11:49   msliu      1.0      

@Description
------------
None
"""

# !/usr/bin/env python
import rospy
from people_msgs.msg import PositionMeasurementArray
# from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class ExtractLegDetector(object):
    """docstring for extract_leg_detector"""

    def __init__(self, arg):
        super(ExtractLegDetector, self).__init__()
        self.arg = arg

        self.pub = rospy.Publisher('neo_marker', Marker, queue_size=10)
        rospy.init_node('neo_talker_leg', anonymous=True)
        # rate = rospy.Rate(10)  # 10hz
        # while not rospy.is_shutdown():
        #     hello_str = "hello world %s" % rospy.get_time()
        #     rospy.loginfo(hello_str)
        #     pub.publish(hello_str)
        #     rate.sleep()

    def leg_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data.people[0].object_id)
        # people = data.people[0]
        item = self.people[0]
        # for item in people:
        print("name: " + item.name)
        print("object_id: " + item.object_id)
        print("Point: " + str(item.pos.x) + " " + str(item.pos.y) + " " + str(item.pos.z))

    def listener_leg_tracker_measurements(self):
        rospy.init_node('neo_listener_leg', anonymous=True)
        rospy.Subscriber("/leg_tracker_measurements", PositionMeasurementArray, self.leg_callback)
        rospy.spin()

    def marker_callback(self, data):
        if data.pose.position.x < 0.2 and data.pose.position.y < 0.2:
            self.pub.publish(data)
        pass

    def listener_visualization_marker(self):
        rospy.init_node('neo_listener_leg', anonymous=True)
        rospy.Subscriber("/leg_tracker_measurements", Marker, self.marker_callback)
        rospy.spin()
        pass

    # def talker_marker(self):
    #     #     pub = rospy.Publisher('neo_marker', Marker, queue_size=10)
    #     #     rospy.init_node('neo_talker_leg', anonymous=True)
    #     #     rate = rospy.Rate(10)  # 10hz
    #     #     while not rospy.is_shutdown():
    #     #         hello_str = "hello world %s" % rospy.get_time()
    #     #         rospy.loginfo(hello_str)
    #     #         pub.publish(hello_str)
    #     #         rate.sleep()


if __name__ == '__main__':
    listener()
