from djitellopy import tello
import numpy as np
import cv2
import math

######## PARAMETERS ###########
interval = 0.25
###############################################

x, y = 500, 500
yaw = 0

me = tello.Tello()
me.connect()
print(me.get_battery())
points = [(0, 0), (0, 0)]

def getDronePosition():
    global x, y, yaw
    speed_x = me.get_speed_x() / 100.0  # Convertendo cm/s para m/s
    speed_y = me.get_speed_y() / 100.0  # Convertendo cm/s para m/s
    yaw = me.get_yaw()

    # Calculando o deslocamento nos eixos X e Y usando trigonometria
    dx = speed_x * interval * math.cos(math.radians(yaw))
    dy = speed_y * interval * math.sin(math.radians(yaw))

    x += dx
    y += dy

    return [x, y, yaw]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)
    cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0] - 500) / 100},{(points[-1][1] - 500) / 100})m',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

while True:
    vals = getDronePosition()
    img = np.zeros((1000, 1000, 3), np.uint8)
    if points[-1][0] != vals[0] or points[-1][1] != vals[1]:
        points.append((int(vals[0]), int(vals[1])))
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
