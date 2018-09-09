#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import math
import numpy as np

aruco = cv2.aruco #arucoライブラリ
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.GridBoard_create(3, 2, 100, 20, dictionary)

def estimate_Pose(img):
    
    in_param = np.loadtxt("in_param.csv", delimiter=",")
    dist = np.loadtxt("dist.csv", delimiter=",")
    img = cv2.undistort(img, in_param, dist)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
    aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
    
    retval, R, t = aruco.estimatePoseBoard(corners, ids, board, in_param, dist)
    aruco.drawAxis(img, in_param, dist, R, t, 150.0)
    print(R)
    print(t)

    cv2.imshow('drawDetectedMarkers', img)
    if cv2.waitKey(0) == 27:
        cv2.destroyAllWindows()

def calc(cam_mtx, xt, yt):

    x0 = cam_mtx[0][3]
    y0 = cam_mtx[1][3]
    z0 = cam_mtx[2][3]
    
    vec = [3]
    vec[0] = cam_mtx[0][0] + cam_mtx[0][1]*x + cam_mtx[0][2]*y
    vec[1] = cam_mtx[1][0] + cam_mtx[1][1]*x + cam_mtx[1][2]*y
    vec[2] = cam_mtx[2][0] + cam_mtx[2][1]*x + cam_mtx[2][2]*y

    if vec[2]<0:
        x = -vec[0] / vec[2] * z0 + x0
        y = -vec[1] / vec[2] * z0 - y0
        length = float(math.sqrt(x**2 + y**2))
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(teta))
        ry = float(length * math.sin(teta))
        
    else:
        x = vec[0]
        y = vec[1]
        length = 10000.0
        theta = float(math.atan2(y, x))
        rx = float(length * math.cos(teta))
        ry = float(length * math.sin(teta))

    print("x = ", rx)
    print("y = ", ry)

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
