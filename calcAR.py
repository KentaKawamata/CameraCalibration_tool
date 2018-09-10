#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import math
import numpy as np

aruco = cv2.aruco #arucoライブラリ
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
#board = aruco.GridBoard_create(3, 2, 50, 10, dictionary)
board = aruco.GridBoard_create(2, 1, 150, 450, dictionary)

def send_XYZ(rote, t, corners, ids):

    rote = np.array(rote)/180
    #R = []
    #for r in rote:
    #    ro, _ = cv2.Rodrigues(r)
    #    R.append(ro)
    R, _ = cv2.Rodrigues(rote)
    #print("R = ", R)

    x = []
    y = []
    for i in range(len(ids)):
        if ids[i] == 0 or ids[i] == 2:
            x.append(corners[i][0][0][0])
            y.append(corners[i][0][0][1])

    #print("corners = \n", corners)
    #print("x = ", x)
    #print("y = ", y)
    rx = [0, 0]
    ry = [0, 0]
    for i in range(len(x)):
        rx[i], ry[i] = calc(x[i], y[i], R, t)
        rx[i] = math.fabs(rx[i])
        ry[i] = math.fabs(ry[i])

    #print("rx = ", rx)
    #print("ry = ", ry)
    strenge_x = (rx[0] - rx[1])**2
    strenge_y = (ry[0] - ry[1])**2
    length = math.sqrt(strenge_x + strenge_y)
    print("length = ", length)

def estimate_Pose(img):
    
    in_param = np.loadtxt("in_param.csv", delimiter=",")
    dist = np.loadtxt("dist.csv", delimiter=",")
    img = cv2.undistort(img, in_param, dist)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
    aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
    
    R, t, _objPoints = aruco.estimatePoseSingleMarkers(corners, 150, in_param, dist)
    #retval, R, t = aruco.estimatePoseBoard(corners, ids, board, in_param, dist)
    
    for i in range(len(ids)):
        print("R = ", R[i])
        aruco.drawAxis(img, in_param, dist, R[i], t[i], 150.0)
        cv2.circle(img, (int(corners[0][0][0][0]),int(corners[0][0][0][1])), 4, (255, 0, 0), -1)
        cv2.circle(img, (int(corners[1][0][0][0]),int(corners[1][0][0][1])), 4, (255, 0, 0), -1)

    t_true = [0, 0, 530]
    R_true = [[0, 0, 45]]
    send_XYZ(R_true, t_true, corners, ids)
    
    cv2.imshow('drawDetectedMarkers', img)
    if cv2.waitKey(0) == 27:
        cv2.destroyAllWindows()


def calc(x, y, R, t):

    x0 = t[0]
    y0 = t[1]
    z0 = t[2]
    #x0 = t[0][0]
    #y0 = t[0][1]
    #z0 = t[0][2]
    
    vec = [0, 0, 0]
    vec[0] = R[0][0] + R[0][1]*x + R[0][2]*y
    vec[1] = R[1][0] + R[1][1]*x + R[1][2]*y
    vec[2] = R[2][0] + R[2][1]*x + R[2][2]*y
    #print("vec[2] = ", vec[2])

    if vec[2]<0:
        x = -vec[0] / vec[2] * z0 + x0
        y = -vec[1] / vec[2] * z0 - y0
        length = float(math.sqrt(x*x + y*y))
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(theta))
        ry = float(length * math.sin(theta))
        
    else:
        x = vec[0]
        y = vec[1]
        length = 10000.0
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(theta))
        ry = float(length * math.sin(theta))

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
