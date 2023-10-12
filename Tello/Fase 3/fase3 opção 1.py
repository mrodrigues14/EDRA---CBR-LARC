from djitellopy import Tello
import cv2
from pyzbar.pyzbar import decode
import time
from djitellopy import Tello
import threading
import keyboard
import numpy as np
import imutils
import os

unique_barcodes = set()

def takeOff():
    tello.takeoff()


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


def flighPlan2():
    mover_passo(312, 26, "esquerda")
    tello.move_down(55)
    mover_passo(312, 26, "direita")
    tello.move_down(55)
    mover_passo(312, 26, "esquerda")
    tello.move_down(65)
    mover_passo(312, 26, "direita")


def returnToBase():
    tello.set_speed(80)
    mover_passo(200, 100, "tras")
    mover_passo(150, 50, "direita")
    mover_passo(400, 100, "tras")


def landing():
    tello.send_control_command("land")


def streaming():
    tello.streamon()

    Dados = 'p'
    if not os.path.exists(Dados):
        print('pasta criada: ', Dados)
        os.makedirs(Dados)

    cap = tello.get_frame_read().frame

    x1, y1 = 200, 100
    x2, y2 = 450, 398

    count = 0
    while True:

        ret, frame = cap.read()
        if ret == False:  break
        imAux = frame.copy()
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        objeto = imAux[y1:y2, x1:x2]
        objeto = imutils.resize(objeto, width=38)
        # print(objeto.shape)

        k = cv2.waitKey(1)
        if k == ord('o'):
            cv2.imwrite(Dados + '/objeto_____{}.jpg'.format(count), objeto)
            print('Imagem capturada: ', 'objeto_{}.jpg'.format(count))
            count = count + 1
        if k == 27:
            break
        cv2.imshow('frame', frame)
        cv2.imshow('objeto', objeto)

    cap.release()
    cv2.destroyAllWindows()

def descer():
    if tello.get_height() < 40:
        tello.land()
def main():
    print(tello.get_battery())
    tello.takeoff()



if __name__ == "__main__":

    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.takeoff()
    a = tello.get_height()
    print(a)
    while True:
        b = tello.get_height()
        print(b)
        if (a - b) > 30:
            tello.emergency()
