#!/usr/bin/pyton3
# -*- encoding: utf-8 -*-
import cv2
import subprocess, sys
import os

class Capture():

  def __init__(self):
      self.img_dir = "/home/kawa/program/CameraCalibration_tool/images/"

  def manual_focus(self):

      try:
          kill_autofocus_left = ['v4l2-ctl', '-d', '/dev/video0', '-c', 'focus_auto=0']
          subprocess.run(kill_autofocus_left)
          kill_autofocus_left = ['v4l2-ctl', '-d', '/dev/video0', '-c', 'focus_auto=0']
          subprocess.run(kill_autofocus_left)
          lock_size_left = ['v4l2-ctl', '-d', '/dev/video0', '--set-fmt-video=width=640,height=360']
          subprocess.run(lock_size_left)
       
          check_camera = ['v4l2-ctl', '--list-ctrls']
          subprocess.run(check_camera)

      except:
          print("-------v4l2 setup command Error !!!!!--------")
          sys.exit()

  def capture_camera(self):

      # カメラをキャプチャする
      cap_left = cv2.VideoCapture(0)


      count=0
      while True:
          ret_left, frame_left = cap_left.read()
          cv2.imshow('test', frame_left)

          k = cv2.waitKey(1) 
          if k==115:
              print("----- save image !!! -----")
              cv2.imwrite(os.path.join(self.img_dir, "img_" + str(count) + ".jpg"), frame_left)
              count+=1

          if k == 27:
              break

      cap_left.release()
      cv2.destroyAllWindows()

  def capture(self):
      #self.manual_focus()
      self.capture_camera()

def main():

    cap = Capture()
    cap.capture()

if __name__ == "__main__":
    main()
