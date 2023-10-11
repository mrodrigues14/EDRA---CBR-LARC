from djitellopy import Tello
import cv2
from pyzbar.pyzbar import decode
import time
from djitellopy import Tello
import threading


def takeOff():
    tello.takeoff()


def adjustHeight():
    tello.set_speed(60)
    while tello.get_height() < targetAltitudeTakeOff:
        altitudeCorrection = targetAltitudeTakeOff - tello.get_height()
        tello.move_up(altitudeCorrection)
        break


def mover_passo(distancia, passo, direcao):
    distancia_passo = int(distancia / passo)

    if (direcao == "esquerda"):
        for i in range(distancia_passo):
            tello.move_left(passo)

    if (direcao == "direita"):
        for i in range(distancia_passo):
            tello.move_right(passo)

    if (direcao == "frente"):
        for i in range(distancia_passo):
            tello.move_forward(passo)

    if (direcao == "tras"):
        for i in range(distancia_passo):
            tello.move_back(passo)


def flightPlan1():
    mover_passo(400, 100, "frente")
    mover_passo(185, 35, "esquerda")
    mover_passo(175, 55, "frente")

    tello.set_speed(30)
    mover_passo(312, 26, "esquerda")
    tello.move_up(60)
    mover_passo(312, 26, "direita")
    tello.move_up(55)
    mover_passo(312, 26, "esquerda")
    tello.move_up(55)
    mover_passo(312, 26, "direita")

    time.sleep(2)

    mover_passo(312, 26, "esquerda")
    tello.move_down(55)
    mover_passo(312, 26, "direita")
    tello.move_down(55)
    mover_passo(312, 26, "esquerda")
    tello.move_down(65)
    mover_passo(312, 26, "direita")


def returnToBase():
    tello.move_up(150)
    tello.set_speed(80)
    tello.move_right(150)
    mover_passo(150, 50, "direita")
    mover_passo(600, 100, "tras")


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
        img = cv2.convertScaleAbs(img, alpha=1.5, beta=25)

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


def main():
    print(tello.get_battery())
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()
    time.sleep(5)
    takeOff()
    time.sleep(3)
    flightPlan1()
    returnToBase()
    time.sleep(3)
    landing()
    streaming_thread.join()


if __name__ == "__main__":

    tello = Tello()
    tello.connect()

    while True:
        try:
            main()
        except:
            continue
        else:
            break