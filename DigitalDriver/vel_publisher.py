#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import pygame
from geometry_msgs.msg import Twist, Vector3

if __name__ == "__main__":

    rospy.init_node("vel_cmd_node")
    r = rospy.Rate(10.0)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    speed = 0.0
    omega = 0.0
    twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, omega))

    pygame.init()
    size = [300, 300]
    screen = pygame.display.set_mode(size)
    clock = pygame.time.Clock()

    class vel():
        def __init__(self):
            self.speed = 0.0
            self.omega = 0.0
            self.twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, omega))


    def con_xbox_vel(vel):
        default_speed = 0.1
        default_omega = 0.1
        car_radius = 0.27
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        # while True:
        #     for event in pygame.event.get():
        #         if event.type == pygame.QUIT:
        #             done = True
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            hats = joystick.get_numhats()
            for i in range(hats):
                hat = joystick.get_hat(i)
                if hat == (1, 0):
                    vel.omega = -default_omega
                    vel.speed = vel.omega * car_radius
                    print("FX right")
                if hat == (-1, 0):
                    vel.omega = default_omega
                    vel.speed = vel.omega * car_radius
                    print("FX left")
                if hat == (0, 1):
                    vel.speed = default_speed
                    vel.omega = 0
                    print("FX up")
                if hat == (0, -1):
                    vel.speed = -default_speed
                    vel.omega = 0
                    print("FX down")
                if hat == (0, 0):
                    vel.speed = 0
                    vel.omega = 0
                    print("Stop")
            pygame.display.flip()
            clock.tick(1)


    ve = vel()

    while not rospy.is_shutdown():
        con_xbox_vel(ve)
        twist = Twist(Vector3(ve.speed, 0, 0), Vector3(0, 0, ve.omega))
        # twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, omega))
        pub.publish(twist)

        r.sleep()