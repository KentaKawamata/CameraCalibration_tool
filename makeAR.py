#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def arGenerator():
    
    fileName0 = "ar0.png"
    fileName2 = "ar2.png"
    AR_size = 1000

    generator0 = aruco.drawMarker(dictionary, 0, AR_size)
    generator2 = aruco.drawMarker(dictionary, 2, AR_size)
    cv2.imwrite(fileName0, generator0)
    cv2.imwrite(fileName2, generator2)

    img = cv2.imread(fileName0)
    cv2.imshow('ArMaker',img)
    cv2.waitKey(0)

def main():

    arGenerator()

if __name__ == "__main__":

    main()
