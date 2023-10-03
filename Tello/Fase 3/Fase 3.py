from djitellopy import Tello
import cv2
from pyzbar.pyzbar import decode
import time
import threading
import keyboard

def main():
    takeOff()
    adjustHeight()
    goToShelf()
    precisionLanding()

def takeOff():
    tello.takeoff()

def adjustHeight():
    if tello.get_height() > 120:
        altitudeDiference = tello.get_height() - 100
        tello.move_down(altitudeDiference)
    else:
        altitudeDiference = -(tello.get_height()) - 100
        tello.move_up(altitudeDiference)
def goToShelf():
    tello.move_forward(700)
    while True:
        img = tello.get_frame_read().frame
        barcodes = decode(img)
        if barcodes:
            break
        tello.move_right(10)

    tello.move_right(400)
    tello.move_up(50)

    while True:
        img = tello.get_frame_read().frame
        barcodes = decode(img)
        if barcodes:
            break
        tello.move_left(10)

    tello.move_left(400)

def precisionLanding():
    tello.land()

def streaming():
    tello.streamon()
    unique_barcodes = set()
    while True:
        print(tello.get_height())
        img = tello.get_frame_read().frame
        barcodes = decode(img)
        for barcode in barcodes:
            if barcode.type == "CODE39":
                barcode_data = barcode.data.decode('utf-8')
                unique_barcodes.add(barcode_data)  # Add to the set of unique barcodes

                # Draw the barcode rectangle and text
                (x, y, w, h) = barcode.rect
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{barcode_data}"
                cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        altitude = tello.get_height()
        altitude_text = f"Altitude: {altitude} cm"
        cv2.putText(img, altitude_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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
    time.sleep(10)
    main()
    streaming_thread.join()
