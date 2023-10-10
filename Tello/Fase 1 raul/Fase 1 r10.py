import time
from djitellopy import Tello

def takeOff():
    tello.takeoff()


def flightToBase():
    tello.set_speed(40)
    tello.move_up(40)
    tello.move_left(25)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(80)
    tello.move_forward(70)
    tello.land()
    tello.takeoff()
    tello.move_up(40)
    tello.move_left(250)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(20)
    tello.move_back(145)
    tello.land()
    tello.takeoff()
    tello.move_right(100)
    tello.move_right(100)
    tello.move_right(75)


def landing():
    tello.takeoff()
    time.sleep(5)
    tello.land()


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.takeoff()
    time.sleep(5)
    flightToBase()
    time.sleep(5)