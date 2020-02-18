import numpy as np
import cv2
import PIL
import os
from cv2 import aruco
import threading as thread
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits import mplot3d
import pandas as pd
import RPi.GPIO as GPIO
import lcddriver
from time import sleep
import math
    


def getArucoLocation(frame, camera_matrix, distortion_coefficients0):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        ret = aruco.estimatePoseSingleMarkers(corners, 20, camera_matrix, distortion_coefficients0)
        rvecs, tvecs = ret[0][0,0,:], ret[1][0,0,:]
        aruco.drawDetectedMarkers(frame,corners)
        aruco.drawAxis(frame, camera_matrix, distortion_coefficients0, rvecs, tvecs, 10)
        str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvecs[0], tvecs[1], tvecs[2])
        cv2.putText(frame, str_position, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.1, (0,255))
        print(str_position)
        return True, rvecs, tvecs, frame
    else:
        return False, False, False , False



def getDeviation(tvecs):
    trajectory_x, trajectory_y, trajectory_z = 0, 0, tvecs[2]
    return math.sqrt(((tvecs[0] - trajectory_x) * (tvecs[0] - trajectory_x)) + ((tvecs[1] - trajectory_y) * (tvecs[1] - trajectory_y)) + ((tvecs[2] - trajectory_z) * (tvecs[2] - trajectory_z)))
    
    



class Calibration:
    
    def __init__(self,main_display):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.display = main_display

    def showArucoMarkers(self):
        fig = plt.figure()
        nx = 8
        ny = 6
        
        for i in range(1, nx*ny+1):
            print(i)
            ax = fig.add_subplot(nx, ny, i)
            img = aruco.drawMarker(self.aruco_dict, i-1, 700)
            plt.imshow(img, cmap = mpl.cm.gray, inerpolation = "nearest")
            ax.axis("off")

        plt.show

    def createCheckerBoard(self):
        board = aruco.CharucoBoard_create(3,3, 1, 0.8, self.aruco_dict)
        imboard = board.draw((4000, 4000))
        return board

    def calibrate_camera(self,allCorners,allIds,imsize):
        display.lcd_display_string("Processing...", 1)
        cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

        distCoeffsInit = np.zeros((5,1))
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL)
        (ret, camera_matrix, distortion_coefficients0,
         rotation_vectors, translation_vectors,
         stdDeviationsIntrinsics, stdDeviationsExtrinsics,
         perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
        
        print("finished")
        display.lcd_clear()
        display.lcd_display_string("Calibration", 1)
        display.lcd_display_string("Successful", 2)
        sleep(2)
        display.lcd_clear()
        
        return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors
        
        

    def read_checkerboard_data(self):
        allCorners = []
        allIds = []
        decimator = 0
        frames_read = 0
        calibartion_video_capture = cv2.VideoCapture(0)
        
        while(frames_read < 20):
            ret, frame = calibartion_video_capture.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            res = cv2.aruco.detectMarkers(gray, self.aruco_dict)
            
            if len(res[0]) > 0 and ret:
                res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray,board)
                if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                    allCorners.append(res2[1])
                    allIds.append(res2[2])
                    frames_read += 1
                    print("{0} out of 30 frame calibrated".format(frames_read))
                    display.lcd_display_string("{0} out of 30 frame".format(frames_read), 1)
                    display.lcd_display_string("calibrated".format(frames_read), 2)
                    sleep(0.1)
                else:
                    print("OpenCV cannot see insintric calibration checkerboard")

                decimator += 1

        display.lcd_clear()
        calibartion_video_capture.release()
        imsize = gray.shape
        return allCorners, allIds, imsize
        print("Finished")
        display.lcd_display_string("Checkerboard", 1)
        display.lcd_display_string("Successful", 2)
        sleep(2)
        display.lcd_clear()
   
    


GPIO.setmode(GPIO.BCM)
#Pin Numbers
ledPin = 20
buttonPin = 21

#GPIO pin setup
GPIO.setup(buttonPin, GPIO.IN, pull_up_down = GPIO.PUD_UP) #Trigger Setup

#LCD setup
display = lcddriver.lcd()

#Display Test
display.lcd_display_string("Welcome to A.T.P.D.A.S", 1)
sleep(2)
display.lcd_clear()

display.lcd_display_string("Time to calibrate", 1)
sleep(2)
display.lcd_clear()
display.lcd_display_string("Point it", 1)
display.lcd_display_string("At the Board", 2)
sleep(2)
display.lcd_clear()

calibration = Calibration(display)

#calibration.showArucoMarkers()
board = calibration.createCheckerBoard()
allCorners, allIds, imsize = calibration.read_checkerboard_data()
ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = calibration.calibrate_camera(allCorners, allIds, imsize)

#shot_video_capture = cv2.VideoCapture(0)

drawn_video_capture = cv2.VideoCapture(0)
buttonPress = True
anotherVar = False

try:
    while True:
        ret2, frame = drawn_video_capture.read()
        display.lcd_clear()
        buttonPress = GPIO.input(buttonPin)
        if ret2:
            valid, rvecs, tvecs, drawn_frame = getArucoLocation(frame, camera_matrix, distortion_coefficients0)
            if buttonPress == False and anotherVar == False:
                if valid:
                    fig = plt.figure()
                    ax = plt.axes(projection='3d')
                    zline = np.linspace(0, tvecs[2])
                    xline = np.linspace(0, 0)
                    yline = np.linspace(0, 0)
                    ax.plot3D(xline, yline, zline, 'gray')
                    zline = np.linspace(tvecs[2], tvecs[2])
                    xline = np.linspace(0, tvecs[0])
                    yline = np.linspace(0, tvecs[1])
                    ax.plot3D(xline, yline, zline, 'green')
                    zline = np.linspace(tvecs[2], 0)
                    xline = np.linspace(tvecs[0], 0)
                    yline = np.linspace(tvecs[1], 0)
                    deviation = getDeviation(tvecs)
                    ax.plot3D(xline, yline, zline, 'blue')
                    if deviation < 10:
                        display.lcd_display_string("YOU HIT", 1)
                    else:
                        display.lcd_display_string("YOU MISSED", 1)
                    sleep(3)
                    display.lcd_clear()
                    display.lcd_display_string("You were off", 1)
                    display.lcd_display_string("by {0} cm".format(deviation), 2)
                    sleep(3)
                    display.lcd_clear()
                    anotherVar = True
            else:
                display.lcd_display_string("Ready to", 1)
                display.lcd_display_string("Fire", 2)
                anotherVar = False
        cv2.imshow('frame', drawn_frame)
        plt.show(block=False)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    plt.show()
finally:
    GPIO.cleanup()
    

                        







