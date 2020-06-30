#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import signal
import time
from xbox360controller import Xbox360Controller

rumble_duration = 50
rumble_strength = 0.5

def on_button_pressed(button):
    if button.name == "button_a":
        # put your function here for event "button_a"
        controller.set_rumble(rumble_strength, rumble_strength, rumble_duration)
        pass
    elif button.name == "button_b":
        # put your function here for event "button_b"
        controller.set_rumble(rumble_strength, rumble_strength, rumble_duration)
        pass
    elif button.name == "button_x":
        # put your function here for event "button_x"
        controller.set_rumble(rumble_strength, rumble_strength, rumble_duration)
        pass
    elif button.name == "button_y":
        # put your function here for event "button_y"
        controller.set_rumble(rumble_strength, rumble_strength, rumble_duration)
        pass
    elif button.name == "button_trigger_l":
        # put your function here for event "button_trigger_l"
        controller.set_rumble(rumble_strength, 0.0, rumble_duration)
        pass
    elif button.name == "button_trigger_r":
        # put your function here for event "button_trigger_r"
        controller.set_rumble(0.0, rumble_strength, rumble_duration)
        pass
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    if button.name == "button_a":
        # put your function here for event "button_a"
        #
        pass
    elif button.name == "button_b":
        # put your function here for event "button_b"
        #
        pass
    elif button.name == "button_x":
        # put your function here for event "button_x"
        #
        pass
    elif button.name == "button_y":
        # put your function here for event "button_y"
        #
        pass
    elif button.name == "button_trigger_l":
        # put your function here for event "button_trigger_l"
        #
        pass
    elif button.name == "button_trigger_r":
        # put your function here for event "button_trigger_r"
        #
        pass
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))


if __name__ == "__main__":
    try:
        with Xbox360Controller(0, axis_threshold=0.2) as controller:
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

            signal.pause()

    except KeyboardInterrupt:
        pass