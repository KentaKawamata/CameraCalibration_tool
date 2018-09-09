#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import glob
import sys

def write_calibrate_param(dist):

    data = []
    with open("calib.txt") as fp:
        try:
            while fp.readline() is True:
                data.append(fp.readline())
        except:
            print("No parameter !!")
            sys.exit()

def read_calib_param():

    data = []
    with open("calib.txt") as fp:
        try:
            while fp.readline() is True:
                data.append(fp.readline())
        except:
            print("No parameter !!")
            sys.exit()

    internal_param = data[0]
    distrtion_param = data[1]

    return internal_param, distrtion_param

def draw_corner(img, corners_sub, ret):
    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (7,6), corners_sub, ret)
    cv2.imshow('img',img)
    cv2.waitKey(500)

def re_draw_corner(name, corners_sub, re_points, ret):
    # Draw and display the corners
    img = cv2.imread(name, 1)
    for i, re_point  in enumerate(re_points):
        point = corners_sub[i]
        cv2.circle(img, (int(point[0][0]),int(point[0][1])), 4, (0, 255, 0), -1)
        cv2.circle(img, (int(re_point[0][0]),int(re_point[0][1])), 4, (0, 0, 255), -1)
    
    cv2.imshow('img',img)
    cv2.waitKey()
    cv2.destroyAllWindows()

def calibrate_images():

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    img_name = []
    
    images = glob.glob('/home/kawa/program/robocup/images/*.jpg')

    check=0    
    for fname in images:
        print(fname)
        img = cv2.imread(fname, 1)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    
        if ret is True:
            check+=1
            corners_sub = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners_sub)
            img_name.append(fname)
    
            img = draw_corner(img, corners_sub, ret)
    
        elif ret is False:
            print("NO")
    
    cv2.destroyAllWindows()
    if check == 0:
        sys.exit()

    ret, in_para, dist, R, t = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    
    
    np.savetxt("in_param.csv", in_para, delimiter = ',', fmt="%0.14f")
    np.savetxt("dist.csv", dist, delimiter = ',', fmt="%0.14f")

    print("t = ", t)
    rote, _ = cv2.Rodrigues(R[0])
    print("rote = ", rote)

    # 再投影誤差による評価
    mean_error = 0
    for i in range(len(img_name)):
        
        re_points, _ = cv2.projectPoints(objpoints[i], R[i], t[i], in_para, dist)
        error = cv2.norm(imgpoints[i], re_points, cv2.NORM_L2) / len(re_points)
        re_draw_corner(img_name[i], imgpoints[i], re_points, 1)
        mean_error += error

    # 0に近い値が望ましい(魚眼レンズの評価には不適？)
    print("total error: ", mean_error/len(objpoints))
 
def main():

    calibrate_images()

if __name__ == "__main__":

    main()
