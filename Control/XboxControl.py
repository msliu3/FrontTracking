import pygame


class xbox_control():

    def __init__(self):
        pygame.init()
        self.size = [300, 300]
        self.screen = pygame.display.set_mode(self.size)
        self.clock = pygame.time.Clock()
        self.speed = 0.0
        self.omega = 0.0
        self.radius = 0.0

    def set_driver(self, control_driver):
        control_driver.speed = self.speed
        control_driver.radius = self.radius
        control_driver.omega = self.omega
        return

    def control_xbox(self, control_driver):
        default_speed = 0.05
        default_omega = 0.1
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()

                axes = joystick.get_numaxes()
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    # if i == 1 and axis == -1.0:
                    #     print("Left up", axis)
                    # if i == 1 and axis > 0.3:
                    #     print("Left down", axis)
                    # if i == 0 and axis == -1:
                    #     print("Left left", axis)
                    # if i == 0 and axis > 0.3:
                    #     print("Left right", axis)
                    # if i == 4 and axis == -1.0:
                    #     print("Right up", axis)
                    # if i == 4 and axis > 0.3:
                    #     print("Right down", axis)
                    # if i == 3 and axis == -1:
                    #     print("Right left", axis)
                    # if i == 3 and axis > 0.3:
                    #     print("Right right", axis)
                    # if i == 2 and axis > 0.3:
                    #     print("LT", axis)
                    # if i == 5 and axis > 0.3:
                    #     print("RT", axis)

                buttons = joystick.get_numbuttons()
                for i in range(buttons):
                    button = joystick.get_button(i)
                    # if i == 0 and button == 1:
                    #     print("A")
                    # if i == 1 and button == 1:
                    #     print("B")
                    # if i == 2 and button == 1:
                    #     print("X")
                    # if i == 3 and button == 1:
                    #     print("Y")
                    # if i == 4 and button == 1:
                    #     print("LB")
                    # if i == 5 and button == 1:
                    #     print("RB")
                    # if i == 6 and button == 1:
                    #     print("BACK")
                    # if i == 7 and button == 1:
                    #     print("START")
                    # if i == 8 and button == 1:
                    #     print("Logitech")
                    # if i == 9 and button == 1:
                    #     print("Left GA")
                    # if i == 10 and button == 1:
                    #     print("Right GA")

                hats = joystick.get_numhats()
                for i in range(hats):
                    hat = joystick.get_hat(i)
                    if hat == (1, 0):
                        self.speed = 0.0
                        self.omega = -default_omega
                        self.radius = 0.0
                        self.set_driver(control_driver)
                        print("FX right")
                    if hat == (-1, 0):
                        self.speed = 0.0
                        self.omega = default_omega
                        self.radius = 0.0
                        self.set_driver(control_driver)
                        print("FX left")
                    if hat == (0, 1):
                        self.speed = default_speed
                        self.omega = 0.0
                        self.radius = 0.0
                        self.set_driver(control_driver)
                        print("FX up")
                    if hat == (0, -1):
                        self.speed = -default_speed
                        self.omega = 0.0
                        self.radius = 0.0
                        self.set_driver(control_driver)
                        print("FX down")
                    if hat == (0, 0):
                        self.speed = 0.0
                        self.omega = 0.0
                        self.radius = 0.0
                        self.set_driver(control_driver)
                        print("Stop")

            pygame.display.flip()
            self.clock.tick(1)

    def con_xbox_vel(self, vel):
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
            self.clock.tick(1)

if __name__ == '__main__':
    xboxc = xbox_control()



