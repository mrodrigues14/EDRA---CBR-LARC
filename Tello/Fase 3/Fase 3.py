from djitellopy import Tello
import cv2
from pyzbar.pyzbar import decode
import time
import threading

current_frame = None
targetAltitudeTakeOff = 100


def takeOff():
    tello.takeoff()
    tello.flip("f")


def adjustHeight():
    tello.set_speed(60)
    while tello.get_height() < targetAltitudeTakeOff:
        altitudeCorrection = targetAltitudeTakeOff - tello.get_height()
        tello.move_up(altitudeCorrection)
        break


def flightPlan1():
    tello.move_forward(400)
    tello.move_left(185)
    tello.move_forward(175)
    tello.set_speed(30)
    tello.move_left(310)
    tello.move_up(65)
    tello.move_right(310)
    tello.move_up(65)
    tello.move_left(310)
    tello.move_up(65)
    tello.move_right(310)


def returnToBase():
    tello.set_speed(80)
    tello.move_right(150)
    tello.move_back(170)
    tello.move_back(400)


def landing():
    tello.send_control_command("land")

def streaming():
    global current_frame
    tello.streamon()
    unique_barcodes = set()
    while True:
        original_img = tello.get_frame_read().frame
        img = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
        barcodes = decode(img)  # Realiza a detecção na imagem original

        # Ajusta o brilho e contraste após a detecção
        img = cv2.convertScaleAbs(img, alpha=1.5, beta=40)

        for barcode in barcodes:
            if barcode.type == "CODE39":
                barcode_data = barcode.data.decode('utf-8')
                unique_barcodes.add(barcode_data)
                (x, y, w, h) = barcode.rect
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{barcode_data}"
                cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the count and list of unique barcodes on the screen
        info_text = f"Quantidade de códigos de barra detectados: {len(unique_barcodes)}"
        cv2.putText(img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y_offset = 60
        for barcode in unique_barcodes:
            cv2.putText(img, barcode, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
    time.sleep(2)
    flightPlan1()
    time.sleep(2)
    returnToBase()
    tello.land()
    streaming_thread.join()
