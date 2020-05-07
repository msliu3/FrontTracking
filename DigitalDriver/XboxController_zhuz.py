#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import signal
import time
from xbox360controller import Xbox360Controller
import rospy
from geometry_msgs.msg import Twist, Vector3

speed = 0.1
omega = 0.3
twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
rumble_duration = 50
rumble_strength = 0.5

def on_button_pressed(button):
    global speed, omega
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
    elif button.name == "button_trigger_l":
        if speed < 1.0:
            speed += 0.05
        if omega < 1.0:
            omega += 0.1
        controller.set_rumble(rumble_strength, 0.0, rumble_duration)
        pass
    elif button.name == "button_trigger_r":
        if speed > 0.1:
            speed -= 0.05
        if omega > 0.1:
            omega -= 0.1
        controller.set_rumble(0.0, rumble_strength, rumble_duration)
    elif button.name == "button_thumb_l":
        #
        controller.set_rumble(rumble_strength, 0.0, rumble_duration)
    elif button.name == "button_thumb_r":
        #
        controller.set_rumble(0.0, rumble_strength, rumble_duration)
        pass
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    print('Button {0} was released'.format(button.name))

    if button.name == "button_a":
        #
        pass
    elif button.name == "button_b":
        #
        pass
    elif button.name == "button_x":
        #
        pass
    elif button.name == "button_y":
        #
        pass
    elif button.name == "button_trigger_l":
        #
        pass
    elif button.name == "button_trigger_r":
        #
        pass
    elif button.name == "button_thumb_l":
        #
        pass
    elif button.name == "button_thumb_r":
        #
        pass


def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))


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
# Left trigger
controller.button_trigger_l.when_pressed = on_button_pressed
controller.button_trigger_l.when_released = on_button_released
# Right trigger
controller.button_trigger_r.when_pressed = on_button_pressed
controller.button_trigger_r.when_released = on_button_released
# Left and right axis move event
controller.axis_l.when_moved = on_axis_moved
controller.axis_r.when_moved = on_axis_moved
# Left thumb
controller.button_thumb_l.when_pressed = on_button_pressed
controller.button_thumb_l.when_released = on_button_released
# Right thumb
controller.button_thumb_r.when_pressed = on_button_pressed
controller.button_thumb_r.when_released = on_button_released

msg = """
Moving around:
          (FORWARD)
              Y   
   (LEFT)X          B(RIGHT)
              A
            (BACK)
            
Left trigger : increase speeds & omega by 0.1
Right trigger: decrease speeds & omega by 0.1
CTRL-C to quit
"""

if __name__ == "__main__":
    print(msg)
    rospy.init_node("xbox_controller_node")
    pub = rospy.Publisher("cmd_vel", Twist)
    rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    while 1:
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