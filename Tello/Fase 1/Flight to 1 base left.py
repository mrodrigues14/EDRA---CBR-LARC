import time
from djitellopy import Tello

def takeOff():
    tello.takeoff()


def flightToBase():
    tello.set_speed(40)
    tello.move_up(140)
    tello.move_left(100)
    tello.move_left(100)
    tello.move_left(50)
    tello.land()
    tello.takeoff()
    tello.move_right(100)
    tello.move_right(100)
    tello.move_right(50)

def landing():
    tello.land()


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.takeoff()
    time.sleep(5)
    flightToBase()
    time.sleep(5)
    landing()