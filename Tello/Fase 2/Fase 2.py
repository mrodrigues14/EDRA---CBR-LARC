from djitellopy import Tello
import cv2
from pyzbar.pyzbar import decode
import time
import threading

current_frame = None


def takeOff():
    tello.takeoff()


def flightPlan():
    tello.set_speed(40)
    tello.move_up(40)
    tello.move_left(25)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(100)
    tello.move_forward(70)
    tello.move_up(60)
    tello.move_left(250)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(100)
    tello.move_back(65)
    tello.move_right(100)
    tello.move_right(100)
    tello.move_right(75)


def landing():
    tello.send_control_command("land")


def streaming():
    global current_frame
    tello.streamon()
    unique_qrcodes = set()  # Set to store unique QR codes
    while True:
        time.sleep(0.1)
        img = tello.get_frame_read().frame
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.convertScaleAbs(img, alpha=1.0, beta=30)
        current_frame = img
        codes = decode(img)
        for code in codes:
            if code.type == "QRCODE":  # Check if the detected code is a QR code
                qr_data = code.data.decode('utf-8')
                unique_qrcodes.add(qr_data)

            (x, y, w, h) = code.rect
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = f"{code.data.decode('utf-8')}"
            cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the count and list of unique QR codes on the screen
        qr_info_text = f"Quantidade de QR codes detectados: {len(unique_qrcodes)}"
        cv2.putText(img, qr_info_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        y_offset = 80

        for qr in unique_qrcodes:
            cv2.putText(img, qr, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            y_offset += 30

        cv2.imshow("frame", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit the streaming
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    time.sleep(8)
    takeOff()
    time.sleep(5)
    flightPlan()
    time.sleep(5)
    streaming_thread.join()
