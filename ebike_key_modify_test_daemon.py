import logging
import socket
import sys
import threading
import time
from logging.handlers import RotatingFileHandler

import RPi.GPIO as GPIO
import numpy as np
import serial

cur_version = sys.version_info
if cur_version >= (3, 0):
    import queue
else:
    import Queue

# communication commands and its default values
com_com = {
    "Mode": "GPIO",
    "Speed": 0,
    "CruiserMode": "Off",
    "RatedVoltage": 60,
    "Volt": 0,
    "Status": "Park",
    "Gear": "None",
    "Beam": "None",
    "TurnSignalLamp": "None",
    "HallMalfunction": "Off",
    "GripShiftMalfunction": "Off",
    "ControllerMalfunction": "Off",
    "OpenPhaseMalfunction": "Off",
    "Button": "None",
    "SignalPeriod": 0,
    "Signal": "Stop",
}

# signal names
var_high_beam = "high_beam "
var_low_beam = "low_beam "
var_turn_left = "turn_left "
var_turn_right = "turn_right"
var_low_gear = "low_gear"
var_medium_gear = "medium_gear"
var_high_gear = "high_gear"
var_speed = "speed"
var_parking = "parking"
var_position = "position"
var_backward_gear = "backward_gear"
var_charging = "charging"
var_one_wire = "one_wire"
var_dangerous = "dangerous"
var_button_plus = "button_plus"
var_button_minus = "button_minus"
var_button_option = "button_option"

# GPIO pins and signals mapping
# pin numbers are BOARD pin number
signal_pin_array = [
    {var_high_beam: 37},
    {var_low_beam: 13},
    {var_turn_left: 16},
    {var_turn_right: 29},
    {var_low_gear: 22},
    {var_medium_gear: 18},
    {var_high_gear: 38},
    {var_parking: 40},
    {var_position: 31},
    {var_backward_gear: 36},
    {var_dangerous: 11},
    {var_charging: 12},
]

speed_pin_array = [
    {var_speed: 7},
]

one_wire_pin_array = [
    {var_one_wire: 15},
]

button_pin_array = [
    {var_button_minus: 35},
    {var_button_option: 33},
    {var_button_plus: 32},
]

com_setting = {}

mode_options = ["1wire", "GPIO"]
cruiser_mode_options = ["On", "Off"]
rated_voltage_options = [36, 48, 60, 72]
rated_voltage_max_speed = [55, 65, 75, 85]
status_options = ["Park", "Ready"]
gear_options = ["High", "Medium", "Low", "None"]
beam_options = ["High", "Low", "None"]
turn_signal_lamp_options = ["Left", "Right", "None"]
hall_malfunction_options = ["On", "Off"]
grip_shift_malfunction_options = ["On", "Off"]
controller_malfunction_options = ["On", "Off"]
open_phase_malfunction_options = ["On", "Off"]
button_options = ["PressPlus", "ReleasePlus", "PressMinus", "ReleaseMinus", "PressOption", "ReleaseOption", "None"]
signal_options = ["Start", "Stop"]

g_pwm = None
g_pwm_period = 61
g_max_speed = 65535
g_valid_max_speed = 75
g_min_speed = 0
g_max_signal_period = 5000
g_min_signal_period = 0
g_max_voltage = 100
g_min_voltage = 0
g_threads = []
g_queue = False
g_lock = None
g_event = None
g_semaphore = None
g_threads_error_flag = False


def get_value_in_dict_from_list(key):
    return [lst for lst in signal_pin_array if key in lst][0].get(key)


def gpio_speed_generation():
    app_log.debug("gpio_speed_generation")
    global g_pwm
    global g_pwm_period
    global g_valid_max_speed

    g_pwm = GPIO.PWM([lst for lst in speed_pin_array if var_speed in lst][0].get(var_speed), g_pwm_period)

    # duty_cycle = float(int(com_setting["Speed"]) * (50.0 - 5.0) / (g_valid_max_speed - 3)  + 5.0)
    duty_cycle = float(int(com_setting["Speed"]) * (50.0 - 4.67) / g_valid_max_speed + 4.67)
    app_log.debug("duty_cycle: %s", duty_cycle)
    if duty_cycle > 100.0:
        duty_cycle = 100.0
    g_pwm.start(duty_cycle)
    time.sleep(0.001)


def gpio_status_generation():
    app_log.debug("gpio_status_generation")
    if com_setting["Status"] == "Park":
        GPIO.output(get_value_in_dict_from_list(var_parking), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_backward_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)
    elif com_setting["Status"] == "Ready":
        GPIO.output(get_value_in_dict_from_list(var_parking), GPIO.HIGH)
        GPIO.output(get_value_in_dict_from_list(var_backward_gear), GPIO.LOW)
        if com_setting["Gear"] == "High":
            GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.HIGH)
        elif com_setting["Gear"] == "Medium":
            GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.HIGH)
            GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)
        elif com_setting["Gear"] == "Low":
            GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.HIGH)
            GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)
        else:
            GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
            GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)


def gpio_gear_generation():
    app_log.debug("gpio_gear_generation")
    if com_setting["Gear"] == "High":
        GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.HIGH)
    elif com_setting["Gear"] == "Medium":
        GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.HIGH)
        GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)
    elif com_setting["Gear"] == "Low":
        GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.HIGH)
        GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)
    else:
        GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)


def gpio_beam_generation():
    app_log.debug("gpio_beam_generation")
    if com_setting["Beam"] == "High":
        GPIO.output(get_value_in_dict_from_list(var_low_beam), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_beam), GPIO.HIGH)
    elif com_setting["Beam"] == "Low":
        GPIO.output(get_value_in_dict_from_list(var_low_beam), GPIO.HIGH)
        GPIO.output(get_value_in_dict_from_list(var_high_beam), GPIO.LOW)
    else:
        GPIO.output(get_value_in_dict_from_list(var_low_beam), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_high_beam), GPIO.LOW)


def gpio_turn_signal_lamp_generation():
    app_log.debug("gpio_turn_signal_lamp_generation")
    if com_setting["TurnSignalLamp"] == "Left":
        GPIO.output(get_value_in_dict_from_list(var_turn_right), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_turn_left), GPIO.HIGH)
    elif com_setting["TurnSignalLamp"] == "Right":
        GPIO.output(get_value_in_dict_from_list(var_turn_right), GPIO.HIGH)
        GPIO.output(get_value_in_dict_from_list(var_turn_left), GPIO.LOW)
    else:
        GPIO.output(get_value_in_dict_from_list(var_turn_right), GPIO.LOW)
        GPIO.output(get_value_in_dict_from_list(var_turn_left), GPIO.LOW)


def gpio_button_signal_generation():
    app_log.debug("gpio_button_signal_generation")
    if com_setting["Button"] == "PressPlus":
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.LOW)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    elif com_setting["Button"] == "ReleasePlus":
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.LOW)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(1)
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    elif com_setting["Button"] == "PressMinus":
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.LOW)
        # time.sleep(0.4)
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    elif com_setting["Button"] == "ReleaseMinus":
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.LOW)
        # time.sleep(1)
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    elif com_setting["Button"] == "PressOption":
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.LOW)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    elif com_setting["Button"] == "ReleaseOption":
        # GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        # GPIO.output(button_pin_array[1][var_button_option], GPIO.LOW)
        # GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(1)
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        # time.sleep(0.4)
    else:
        GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
        GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
        GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
        time.sleep(0.4)


def gpio_speed_signal_termination():
    global g_pwm
    global g_pwm_period

    app_log.debug("gpio_speed_signal_termination")
    g_pwm.ChangeDutyCycle(5.0)
    time.sleep(2 / g_pwm_period)
    g_pwm.stop()


def gpio_status_signal_termination():
    app_log.debug("gpio_status_signal_termination")
    GPIO.output(get_value_in_dict_from_list(var_parking), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_backward_gear), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)


def gpio_gear_signal_termination():
    app_log.debug("gpio_gear_signal_termination")
    GPIO.output(get_value_in_dict_from_list(var_low_gear), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_medium_gear), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_high_gear), GPIO.LOW)


def gpio_beam_signal_termination():
    app_log.debug("gpio_beam_signal_termination")
    GPIO.output(get_value_in_dict_from_list(var_low_beam), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_high_beam), GPIO.LOW)


def gpio_turn_signal_lamp_termination():
    app_log.debug("gpio_turn_signal_lamp_termination")
    GPIO.output(get_value_in_dict_from_list(var_turn_right), GPIO.LOW)
    GPIO.output(get_value_in_dict_from_list(var_turn_left), GPIO.LOW)


def gpio_button_signal_termination():
    app_log.debug("gpio_button_signal_termination")
    GPIO.output(button_pin_array[2][var_button_plus], GPIO.HIGH)
    GPIO.output(button_pin_array[1][var_button_option], GPIO.HIGH)
    GPIO.output(button_pin_array[0][var_button_minus], GPIO.HIGH)
    # time.sleep(0.4)


def one_wire_signal_generation(status, app_log):
    global g_lock
    global g_pwm

    app_log.debug(status + " generating signal on 1wire")
    app_log.info("1wire not implemented")
    g_lock.acquire()
    g_lock.release()


def gpio_signal_generation(status, app_log):
    global g_lock

    app_log.debug(status + " generating signal on gpio")

    g_lock.acquire()
    if status == "Start":
        app_log.debug(com_setting)
        # speed generation
        gpio_speed_generation()
        # voltage generation
        # volt not implemented
        # status generation
        gpio_status_generation()
        # gear generation
        gpio_gear_generation()
        # beam generation
        gpio_beam_generation()
        # turn signal lamp generation
        gpio_turn_signal_lamp_generation()
    elif status == "Stop":
        # speed set to 0
        gpio_speed_signal_termination()
        # status set default
        gpio_status_signal_termination()
        # gear set default
        gpio_gear_signal_termination()
        # beam set default
        gpio_beam_signal_termination()
        # turn signal lamp set default
        gpio_turn_signal_lamp_termination()
    else:
        app_log.error("error signal value")
    g_lock.release()


def signal_generation_service(app_log):
    global g_semaphore
    global g_event
    global g_queue
    global g_threads_error_flag

    app_log.debug("signal_generation_service")
    try:
        while not g_threads_error_flag:
            app_log.debug("waiting for event")
            g_event.wait()
            app_log.debug("event set")
            command = g_queue.get()
            app_log.debug("queue length is :%d", g_queue.qsize())
            app_log.debug("command: %s", command)

            if command == "Mode":
                # restart signal generation when mode changed
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                if com_setting["Signal"] == "Start":
                    if com_setting["Mode"] == "1wire":
                        one_wire_signal_generation("Stop", app_log)
                        one_wire_signal_generation("Start", app_log)
                    else:
                        gpio_signal_generation("Stop", app_log)
                        gpio_signal_generation("Start", app_log)
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "RatedVoltage":
                # restart signal generation when rated voltage changed
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                if com_setting["Signal"] == "Start":
                    if com_setting["Mode"] == "1wire":
                        one_wire_signal_generation("Stop", app_log)
                        one_wire_signal_generation("Start", app_log)
                    else:
                        gpio_signal_generation("Stop", app_log)
                        gpio_signal_generation("Start", app_log)
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Speed":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Signal"] == "Start":
                    if com_setting["Mode"] == "GPIO":
                        global g_pwm
                        global g_pwm_period
                        global g_valid_max_speed
                        if g_pwm is not None:
                            # duty_cycle = float(int(com_setting[command]) * (50.0 ) / (g_valid_max_speed) + 5.0)
                            duty_cycle = float(int(com_setting["Speed"]) * (50.0 - 4.67) / g_valid_max_speed + 4.67)
                            app_log.debug("duty_cycle: %s", duty_cycle)
                            if duty_cycle > 100.0:
                                duty_cycle = 100.0
                                app_log.debug("duty_cycle: %s", duty_cycle)
                            g_pwm.ChangeDutyCycle(duty_cycle)
                            time.sleep(2 / g_pwm_period)
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "SignalPeriod":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Signal":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Mode"] == "1wire":
                    one_wire_signal_generation(com_setting[command], app_log)
                else:
                    gpio_signal_generation(com_setting[command], app_log)
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "CruiserMode":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                one_wire_signal_generation(com_setting[command], app_log)
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Volt":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                app_log.info("volt not implemented")
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Status":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Signal"] == "Start":
                    gpio_status_generation()
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Gear":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Signal"] == "Start":
                    gpio_gear_generation()
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Beam":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Signal"] == "Start":
                    gpio_beam_generation()
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "TurnSignalLamp":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                if com_setting["Signal"] == "Start":
                    gpio_turn_signal_lamp_generation()
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "HallMalfunction":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                app_log.info("hall malfunction not implemented")
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "GripShiftMalfunction":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                app_log.info("grip shift malfunction not implemented")
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "ControllerMalfunction":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                app_log.info("controller malfunction not implemented")
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "OpenPhaseMalfunction":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                app_log.info("open phase malfunction not implemented")
                g_semaphore.release()
                app_log.debug("semaphore released")
            elif command == "Button":
                app_log.debug("waiting for semaphore")
                g_semaphore.acquire()
                app_log.debug("semaphore acquired")
                app_log.debug("com_setting[" + command + "]: %s", str(com_setting[command]))
                gpio_button_signal_generation()
                g_semaphore.release()
                app_log.debug("semaphore released")
            else:
                app_log.error("unrecognized command:%s", command)

            g_queue.task_done()
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.clear()
            app_log.debug("event cleared")
    except Exception as e:
        g_threads_error_flag = True
        raise e


def mode_setting(command, value):
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))

    if value in mode_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        app_log.warning("set mode to GPIO as default")
        com_setting["Mode"] = "GPIO"
        return False


def gpio_signal_setting(command, value):
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    if command == "Status" and value in status_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    elif command == "Gear" and value in gear_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    elif command == "Beam" and value in beam_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    elif command == "TurnSignalLamp" and value in turn_signal_lamp_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def speed_setting(command, value):
    global g_max_speed
    global g_min_speed
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    if value.isdigit() and g_min_speed <= int(value) <= g_max_speed:
        if com_setting[command] != int(value):
            com_setting[command] = int(value)
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def button_setting(command, value):
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    if value in button_options:
        com_setting[command] = value
        g_queue.put(command)
        app_log.debug("queue length is :%d", g_queue.qsize())
        g_event.set()
        app_log.debug("set event")
        app_log.debug("waiting for semaphore")
        g_semaphore.acquire()
        app_log.debug("semaphore acquired")
        g_semaphore.release()
        app_log.debug("semaphore released")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def one_wire_setting(command, value):
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    app_log.info("1wire not implemented")
    if value in cruiser_mode_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def rated_voltage_setting(command, value):
    global g_valid_max_speed
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    app_log.info("rated_voltage not implemented")
    if value.isdigit() and int(value) in rated_voltage_options:
        if com_setting[command] != int(value):
            for i in range(len(rated_voltage_options)):
                app_log.debug("rate_voltage:" + str(rated_voltage_options[i]) + " i:" + str(i))
                if int(value) == rated_voltage_options[i]:
                    g_valid_max_speed = rated_voltage_max_speed[i]
                    app_log.debug("max valid speed:" + str(g_valid_max_speed))
            g_queue.put(command)
            g_event.set()
            app_log.debug("set event")
            app_log.debug("queue length is :%d", g_queue.qsize())
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def voltage_setting(command, value):
    global g_max_voltage
    global g_min_voltage
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    app_log.info("voltage not implemented")

    if value.isdigit() and g_min_voltage <= int(value) <= g_max_voltage:
        if com_setting[command] != int(value):
            com_setting[command] = int(value)
            g_queue.put(command)
            g_event.set()
            app_log.debug("set event")
            app_log.debug("queue length is :%d", g_queue.qsize())
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def signal_period_setting(command, value):
    global g_max_signal_period
    global g_min_signal_period
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    if value.isdigit() and g_min_signal_period <= int(value) <= g_max_signal_period:
        if com_setting[command] != int(value):
            com_setting[command] = int(value)
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


def signal_generation(command, value):
    global g_queue
    global g_semaphore

    app_log.debug("command, value:" + command + ", " + str(value))
    if value in signal_options:
        if com_setting[command] != value:
            com_setting[command] = value
            g_queue.put(command)
            app_log.debug("queue length is :%d", g_queue.qsize())
            g_event.set()
            app_log.debug("set event")
            app_log.debug("waiting for semaphore")
            g_semaphore.acquire()
            app_log.debug("semaphore acquired")
            g_semaphore.release()
            app_log.debug("semaphore released")
        else:
            app_log.warning("same " + command + " value:" + value + ". doing nothing")
        return True
    else:
        app_log.error(command + " value: " + str(value) + " not valid")
        return False


signal_function = {
    "Mode": mode_setting,
    "Speed": speed_setting,
    "CruiserMode": one_wire_setting,
    "RatedVoltage": rated_voltage_setting,
    "Volt": voltage_setting,
    "Status": gpio_signal_setting,
    "Gear": gpio_signal_setting,
    "Beam": gpio_signal_setting,
    "TurnSignalLamp": gpio_signal_setting,
    "HallMalfunction": one_wire_setting,
    "GripShiftMalfunction": one_wire_setting,
    "ControllerMalfunction": one_wire_setting,
    "OpenPhaseMalfunction": one_wire_setting,
    "Button": button_setting,
    "SignalPeriod": signal_period_setting,
    "Signal": signal_generation,
}


def init_logging():
    log_formatter = logging.Formatter('%(asctime)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s')
    log_file = 'ebike_control_board.log'

    file_handler = RotatingFileHandler(log_file,
                                       mode='a',
                                       maxBytes=5 * 1024 * 1024,
                                       backupCount=2,
                                       encoding=None,
                                       delay=0)

    file_handler.setFormatter(log_formatter)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(log_formatter)

    app_log = logging.getLogger('root')
    app_log.setLevel(logging.DEBUG)

    app_log.addHandler(file_handler)
    # app_log.addHandler(console_handler)
    return app_log


def init_global_variables():
    global g_pwm
    global g_pwm_period
    global g_max_speed
    global g_valid_max_speed
    global g_min_speed
    global g_min_signal_period
    global g_max_voltage
    global g_min_voltage
    global g_threads
    global g_queue
    global g_lock
    global g_semaphore
    global g_event

    for command, value in com_com.items():
        com_setting[command] = com_com[command]
    g_pwm = None
    g_pwm_period = 61
    g_max_speed = 65535
    g_valid_max_speed = 75
    g_min_speed = 0
    g_min_signal_period = 0
    g_max_voltage = 100
    g_min_voltage = 0
    g_threads = []
    import sys
    if sys.version_info >= (3, 0):
        g_queue = queue.Queue()
    else:
        g_queue = Queue.Queue()

    g_lock = threading.Lock()
    g_event = threading.Event()
    g_semaphore = threading.Semaphore()


def init_signal_generation_service(app_log):
    global g_threads
    global g_queue
    global g_lock
    global g_threads_error_flag

    g_threads_error_flag = False

    t = threading.Thread(target=signal_generation_service, args=(app_log,))
    t.setDaemon(True)
    g_threads.append(t)

    t.start()


def init_gpio_pins(app_log):
    global g_lock
    g_lock.acquire()
    GPIO.setmode(GPIO.BOARD)
    for pin_pair in signal_pin_array:
        for pin, pin_num in pin_pair.items():
            GPIO.setup(pin_num, GPIO.OUT)
    for pin_pair in button_pin_array:
        for pin, pin_num in pin_pair.items():
            GPIO.setup(pin_num, GPIO.OUT)
    for pin_pair in speed_pin_array:
        for pin, pin_num in pin_pair.items():
            GPIO.setup(pin_num, GPIO.OUT)
    for pin_pair in one_wire_pin_array:
        for pin, pin_num in pin_pair.items():
            GPIO.setup(pin_num, GPIO.OUT)
    g_lock.release()


def init_serial(app_log):
    try:
        ser_port = serial.Serial(port='/dev/ttyUSB0',
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
    except serial.SerialException:
        app_log.error("serial exception detected")
        raise serial.SerialException
    else:
        if ser_port.isOpen():
            app_log.info("serial open")
            return ser_port
        else:
            app_log.error("serial already in use")
            return None


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    ip = s.getsockname()
    app_log.debug(ip)
    s.close()
    return ip[0]


def check_signal_gpio_pins(ser_port):
    # set each gpio to high, hold on 5 seconds, then set to low and wait
    # 3 seconds.
    for pin_pair in signal_pin_array:
        for pin, pin_num in pin_pair.items():
            ser_port.write(("pin:" + pin + " " + str(pin_num) + " start to set high").encode('utf-8'))
            GPIO.output(pin_num, GPIO.HIGH)
            time.sleep(3)
            ser_port.write(("pin:" + pin + " " + str(pin_num) + " start to set low").encode('utf-8'))
            GPIO.output(pin_num, GPIO.LOW)
            time.sleep(1)


def speed_sweeping(ser_port):
    pwm = GPIO.PWM(speed_pin_array[0][var_speed], 61)
    ser_port.write(("start to speed sweeping from minimum to maximum" + "\r\n").encode('utf-8'))
    pwm.start(0.0)
    time.sleep(2 / 61)
    pwm.ChangeDutyCycle(45.0)
    time.sleep(2 / 61)
    step = (50.0 - 5.0) / 75
    for i in np.arange(5.0, 50.0, step):
        ser_port.write(("duty cycle:" + str(i) + " - speed:" + str((i - 5.0) / step) + "\r\n").encode('utf-8'))
        pwm.ChangeDutyCycle(i)
        time.sleep(0.1)

    ser_port.write(("start to speed sweeping from maximum to minimum" + "\r\n").encode('utf-8'))
    for i in np.arange(50.0, 5.0, -step):
        ser_port.write(("duty cycle:" + str(i) + " - speed:" + str((i - 5.0) / step) + "\r\n").encode('utf-8'))
        pwm.ChangeDutyCycle(i)
        time.sleep(0.1)

    ser_port.write(("duty cycle:" + str(5.0) + " - speed:" + str((5.0 - 5.0) / step) + "\r\n").encode('utf-8'))
    pwm.ChangeDutyCycle(5.0)
    time.sleep(2 / 61)
    pwm.stop()


def button_check(ser_port):
    ser_port.write("start to button_check".encode('utf-8'))
    ser_port.write("long press option button".encode('utf-8'))
    for pin, pin_num in button_pin_array[1].items():
        GPIO.output(pin_num, GPIO.LOW)
        time.sleep(1)
        GPIO.output(pin_num, GPIO.HIGH)
        time.sleep(1)
    ser_port.write("press + button".encode('utf-8'))
    for pin, pin_num in button_pin_array[2].items():
        GPIO.output(pin_num, GPIO.LOW)
        time.sleep(0.6)
        GPIO.output(pin_num, GPIO.HIGH)
        time.sleep(1)
    ser_port.write("press - button".encode('utf-8'))
    for pin, pin_num in button_pin_array[0].items():
        GPIO.output(pin_num, GPIO.LOW)
        time.sleep(0.6)
        GPIO.output(pin_num, GPIO.HIGH)
        time.sleep(1)
    ser_port.write("press option button".encode('utf-8'))
    for pin, pin_num in button_pin_array[1].items():
        GPIO.output(pin_num, GPIO.LOW)
        time.sleep(0.6)
        GPIO.output(pin_num, GPIO.HIGH)
        time.sleep(1)


def launch_daemon(ser_port, app_log):
    ser_port.flushInput()
    ser_port.flushOutput()
    try:
        in_buffer_bytes = ser_port.inWaiting()
    except serial.SerialException:
        app_log.error("serial exception detected")
        raise serial.SerialException
    except Exception as e:
        app_log.error(e)
        raise e

    if in_buffer_bytes > 0:
        app_log.debug("in_buffer_bytes:" + str(in_buffer_bytes))
    while not g_threads_error_flag:
        if in_buffer_bytes > 0:
            in_bytes = None
            try:
                in_bytes = ser_port.readline()
            except Exception as e:
                app_log.error(e)
                raise e

            in_bytes = in_bytes.decode('utf-8')
            app_log.debug("in_bytes:" + in_bytes)
            in_bytes = in_bytes.rstrip()
            app_log.debug("in_bytes after rstrip:" + in_bytes)

            if in_bytes != '':
                app_log.debug("<<" + in_bytes)
                if in_bytes.find('=') >= 0:
                    command = in_bytes[:in_bytes.index('=')]
                    value = in_bytes[in_bytes.index('=') + 1:]
                    if com_com.get(command) is not None:
                        app_log.info("Command is: " + command)
                        app_log.info("Value is:" + str(value))
                        if signal_function.get(command)(command, value):
                            app_log.info("Result=OK")
                            try:
                                ser_port.flushOutput()
                                ser_port.write(("Result=OK" + "\r\n").encode('utf-8'))
                            except Exception as e:
                                app_log.error(e)
                                raise e
                elif in_bytes.find('IP') >= 0:
                    app_log.debug('ask ip')
                    try:
                        ser_port.flushOutput()
                        ser_port.write((str(get_ip_address('lo')) + "\r\n").encode('utf-8'))
                    except Exception as e:
                        app_log.error(e)
                        raise e
                elif in_bytes.find('SHUTDOWN') >= 0:
                    app_log.debug('>>shutdown raspberry pi')
                    try:
                        ser_port.flushOutput()
                        ser_port.write(('shutdown raspberry pi' + "\r\n").encode('utf-8'))
                    except Exception as e:
                        app_log.error(e)
                        raise e
                    import subprocess
                    command = '/usr/bin/sudo /sbin/shutdown now'
                    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
                    output = process.communicate()[0]
                elif in_bytes.find('TAKESNAPSHOT') >= 0:
                    app_log.debug('take snap shot by camera on raspberry pi')
                    import cv2
                    cap = cv2.VideoCapture(0)
                    ret, im = cap.read()
                    cv2.imwrite('snapshot.jpg', im)
                    try:
                        ser_port.flushOutput()
                        ser_port.write(('ready to send file' + "\r\n").encode('utf-8'))
                        app_log.debug('>>ready to send file')
                    except Exception as e:
                        app_log.error(e)
                        cap.release()
                        raise e
                    else:
                        while ser_port.readline().find('ready to receive file') < 0:
                            pass
                        app_log.debug('ready to receive file')
                        fd = open('snapshot.jpg', "rb")
                        ser_port.write(fd.read())
                        fd.close()
                        ser_port.write('\n<<EOF>>\n')
                        cap.release()
                        app_log.debug('sending file finished')
                elif in_bytes.find('SIGNALCHECK') >= 0:
                    check_signal_gpio_pins(ser_port)
                    speed_sweeping(ser_port)
                    button_check(ser_port)
                else:
                    app_log.info(">>Result=Fail")
                    try:
                        ser_port.flushOutput()
                        ser_port.write(("Result=Fail" + "\r\n").encode('utf-8'))
                    except Exception as e:
                        app_log.error(e)
                        raise e

        time.sleep(0.1)
        try:
            in_buffer_bytes = ser_port.inWaiting()
        except serial.SerialException:
            app_log.error("serial exception detected")
            raise serial.SerialException
        except Exception as e:
            app_log.error(e)
            raise e
        if in_buffer_bytes > 0:
            app_log.debug("in_buffer_bytes:" + str(in_buffer_bytes))


if __name__ == '__main__':
    app_log = init_logging()
    init_global_variables()
    init_signal_generation_service(app_log)
    init_gpio_pins(app_log)

    serial_port = init_serial(app_log)
    if serial_port is not None:
        launch_daemon(serial_port, app_log)
