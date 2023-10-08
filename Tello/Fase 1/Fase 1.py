import time

import cv2
import torch
from djitellopy import Tello
import threading


targetAltitude = 100


def takeOff():
    tello.takeoff()

def adjustHeight():
    while tello.get_height() < targetAltitude:
        altitudeCorrection = targetAltitude - tello.get_height()
        tello.move_up(altitudeCorrection)
        break


def flightToBase():
    tello.set_speed(80)
    tello.move_forward(400)
    tello.move_forward(190)
    tello.land()
    tello.takeoff()
    tello.rotate_counter_clockwise(90)
    tello.move_forward(250)
    tello.rotate_counter_clockwise(90)
    tello.move_forward(400)
    tello.move_forward(200)
    tello.land()
    tello.takeoff()
    tello.rotate_counter_clockwise(90)
    tello.move_forward(275)
    tello.land()


def testeLanding1():
    tello.takeoff()
    time.sleep(5)
    tello.land()

def testeLanding2():
    tello.takeoff()
    time.sleep(5)
    tello.send_control_command("land")


def streaming():
    tello.streamon()


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.takeoff()
    time.sleep(5)
    adjustHeight()
    time.sleep(5)
    flightToBase()

    tello.turn_motor_off()