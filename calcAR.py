#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import math
import numpy as np

aruco = cv2.aruco #arucoライブラリ
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.GridBoard_create(3, 2, 50, 10, dictionary)

def send_XYZ(R, t, corners, ids):

    x = []
    y = []
    for i in range(len(ids)):
        if i == 0 or i == 2:
            x.append(corners[i][0][0][0])
            y.append(corners[i][0][0][1])

    print("x = ", x)
    print("y = ", y)
    rx = [0, 0]
    ry = [0, 0]
    for i in range(len(x)):
        rx[i], ry[i] = calc(x[i], y[i], R, t)

    strenge_x = (x[0] - x[1])**2
    strenge_y = (y[0] - y[1])**2
    length = math.sqrt(strenge_x + strenge_y)
    print("length = ", length)

def estimate_Pose(img):
    
    in_param = np.loadtxt("in_param.csv", delimiter=",")
    dist = np.loadtxt("dist.csv", delimiter=",")
    img = cv2.undistort(img, in_param, dist)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
    aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
    
    retval, R, t = aruco.estimatePoseBoard(corners, ids, board, in_param, dist)
    aruco.drawAxis(img, in_param, dist, R, t, 150.0)

    rote, _ = cv2.Rodrigues(R)
    send_XYZ(rote, t, corners, ids)
    
    cv2.imshow('drawDetectedMarkers', img)
    if cv2.waitKey(0) == 27:
        cv2.destroyAllWindows()

def calc(x, y, R, t):

    print(t)
    x0 = t[0]
    y0 = t[1]
    z0 = t[2]
    
    vec = [0, 0, 0]
    vec[0] = R[0][0] + R[0][1]*x + R[0][2]*y
    vec[1] = R[1][0] + R[1][1]*x + R[1][2]*y
    vec[2] = R[2][0] + R[2][1]*x + R[2][2]*y

    if vec[2]<0:
        x = -vec[0] / vec[2] * z0 + x0
        y = -vec[1] / vec[2] * z0 - y0
        length = float(math.sqrt(x**2 + y**2))
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(theta))
        ry = float(length * math.sin(theta))
        
    else:
        x = vec[0]
        y = vec[1]
        length = 10000.0
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(teta))
        ry = float(length * math.sin(teta))

    return rx, ry

def arReader():
    cap = cv2.VideoCapture(0)

    while True:

        ret, frame = cap.read()
        img = cv2.resize(frame, (640, 480))

        #corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary) #マーカを検出
        #print(corners)

        #aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
        cv2.imshow('drawDetectedMarkers', img)
        
        if cv2.waitKey(100) == 0x20:
           estimate_Pose(img)

        if cv2.waitKey(100) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    arReader()

if __name__ == "__main__":
    main()
