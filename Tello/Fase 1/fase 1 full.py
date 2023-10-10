import time
from djitellopy import Tello
import cv2
import threading

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
    tello.move_forward(70)
    tello.land()
    tello.takeoff()
    tello.move_up(40)
    tello.move_left(250)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(45)
    tello.land()
    tello.takeoff()
    tello.move_right(100)
    tello.move_right(100)
    tello.move_right(75)


def streaming():
    global current_frame
    tello.streamon()
    while True:
        original_img = tello.get_frame_read().frame
        img = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
        img = cv2.convertScaleAbs(img, alpha=1.5, beta=40)
        cv2.imshow("frame", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit the streaming
            break
    cv2.destroyAllWindows()


def landing():
    tello.send_control_command("land")

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    time.sleep(8)
    tello.takeoff()
    time.sleep(5)
    flightToBase()
    time.sleep(5)
    landing()