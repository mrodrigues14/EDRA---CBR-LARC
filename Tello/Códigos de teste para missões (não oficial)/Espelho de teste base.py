import time
from djitellopy import Tello
import cv2
import threading
from pyzbar.pyzbar import decode


def takeOff():
    tello.takeoff()

altura_operacional = 100 #VERIFICAR ALTURA ANTES DE INICIAR QUALQUER MISSÃO

coordenadas_home = [75,100,25]

posicao_base1 = [100, 700, altura_operacional]

posicao_base2 = [350, 100, altura_operacional]


coordenadas_atuais = coordenadas_home

def diferenca(coordenadas_atuais, coordenadas_desejadas):
    dif = len(coordenadas_desejadas) * [0]

    if len(coordenadas_desejadas) > len(coordenadas_atuais):
        dif[2] = coordenadas_desejadas[2]

    for i in range(len(coordenadas_desejadas)):
        dif[i] = coordenadas_desejadas[i] - coordenadas_atuais[i]

    return(dif)

def mover_passo(distancia, passo, direcao):

    distancia_passo = int(distancia/passo)

    if(direcao == "esquerda"):
        for i in range(distancia_passo):
            tello.move_left(passo)
            # print("esquerda: ", passo)

    if (direcao == "direita"):
        for i in range(distancia_passo):
            tello.move_right(passo)
            # print("direita: ", passo)

    if (direcao == "frente"):
        for i in range(distancia_passo):
            tello.move_forward(passo)
            # print("frente: ", passo)

    if (direcao == "tras"):
        for i in range(distancia_passo):
            tello.move_back(passo)
            # print("trás: ", passo)

def vai_para(origem, destino, coordenada_passo = 20):

    direcao = diferenca(origem , destino)
    print(direcao)

    if direcao[2] > 0:
        tello.move_up(direcao[2])
        # print("cima: ", direcao[2])

    if direcao[0] > 0:
        # print("esquerda total: ", direcao[0])
        mover_passo(direcao[0], coordenada_passo, "esquerda")

    if direcao[0] < 0:
        # print("direita total: ", direcao[0] * -1)
        mover_passo(direcao[0]*-1, coordenada_passo, "direita")

    if direcao[1] > 0:
        # print("frente total: ", direcao[1])
        mover_passo(direcao[1], 100, "frente")

    if direcao[1] < 0:
        # print("trás total: ", direcao[1] * -1)
        mover_passo(direcao[1]*-1, 100, "tras")

    # if direcao[2] < 0:
    #     tello.move_down((direcao[2]*-1))
    #     # print("baixo: ", direcao[2]*-1)

def flightToBase():
    tello.set_speed(40)
    vai_para(coordenadas_home,posicao_base1, 25)
    tello.move_up(50)
    vai_para(posicao_base1, posicao_base2, 50)
    vai_para(posicao_base2, coordenadas_home, 25)


def landing():
    tello.land()

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
    time.sleep(6)
    tello.takeoff()
    time.sleep(5)
    flightToBase()
    time.sleep(5)
    landing()
    streaming_thread.join()