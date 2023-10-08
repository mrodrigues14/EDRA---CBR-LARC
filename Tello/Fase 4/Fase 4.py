from djitellopy import Tello
import cv2
import time
import threading

current_frame = None
targetAltitudeTakeOff = 120
plataformaClassif = cv2.CascadeClassifier('cascade.xml')

def takeOff():
    tello.takeoff()

def adjustHeight():
    while tello.get_height() < targetAltitudeTakeOff:
        altitudeCorrection = targetAltitudeTakeOff - tello.get_height()
        tello.move_up(altitudeCorrection)
        break

def flightPlan1():
    tello.move_up()

def flightPlan1():


def landing():
    tello.send_control_command("land")

def streaming():
    global current_frame
    tello.streamon()
    while True:
        img = tello.get_frame_read().frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        toy = plataformaClassif.detectMultiScale(gray,
                                                 scaleFactor=9,
                                                 minNeighbors=91,
                                                 minSize=(70, 78))

        for (x, y, w, h) in toy:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, 'Plataforma', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("frame", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit the streaming
            break

    cv2.destroyAllWindows()

def main():
    takeOff()

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    time.sleep(8)
    streaming_thread.join()
