#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import signal
from xbox360controller import Xbox360Controller
import time
import rospy
from geometry_msgs.msg import Twist, Vector3


speed = 0.1
omega = 0.5
twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


def on_button_pressed(button):
    if button.name == "button_a":
        twist.linear.x = -speed
        twist.angular.z = 0.0
    elif button.name == "button_b":
        twist.linear.x = 0.0
        twist.angular.z = -omega
    elif button.name == "button_x":
        twist.linear.x = 0.0
        twist.angular.z = omega
    elif button.name == "button_y":
        twist.linear.x = speed
        twist.angular.z = 0.0
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    # print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    pass


if __name__ == "__main__":
    rospy.init_node("xbox_controller_node")
    pub = rospy.Publisher("cmd_vel", Twist)
    rate = rospy.Rate(10)

    controller = Xbox360Controller(0, axis_threshold=0.2)
    # Button A events
    controller.button_a.when_pressed = on_button_pressed
    controller.button_a.when_released = on_button_released
    # Button B events
    controller.button_b.when_pressed = on_button_pressed
    controller.button_b.when_released = on_button_released
    # Button X events
    controller.button_x.when_pressed = on_button_pressed
    controller.button_x.when_released = on_button_released
    # Button y events
    controller.button_y.when_pressed = on_button_pressed
    controller.button_y.when_released = on_button_released

    # Left and right axis move event
    controller.axis_l.when_moved = on_axis_moved
    controller.axis_r.when_moved = on_axis_moved

    while not rospy.is_shutdown():
        # while 1:
        pub.publish(twist)
        rate.sleep()

    # try:
    #     with Xbox360Controller(0, axis_threshold=0.2) as controller:
    #         # Button A events
    #         controller.button_a.when_pressed = on_button_pressed
    #         controller.button_a.when_released = on_button_released
    #         # Button B events
    #         controller.button_b.when_pressed = on_button_pressed
    #         controller.button_b.when_released = on_button_released
    #         # Button X events
    #         controller.button_x.when_pressed = on_button_pressed
    #         controller.button_x.when_released = on_button_released
    #         # Button y events
    #         controller.button_y.when_pressed = on_button_pressed
    #         controller.button_y.when_released = on_button_released
    #
    #         # Left and right axis move event
    #         controller.axis_l.when_moved = on_axis_moved
    #         controller.axis_r.when_moved = on_axis_moved
    #
    #         signal.pause()
    #
    # except KeyboardInterrupt:
    #     pass