#Sohan Shanbhag - 10.02.2022
#Language used - Python, Main package - opencv
#servo motor connected with arduino rotates as fingers are move further away from each other
#with this we can control the angle of movement of the servo motor with our fingers without touching the keyboard

#importing packages
import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np
import pyfirmata
import serial
import time
from pyfirmata import SERVO
import math

#initializing camera used for vision
cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

#initializing detector to find hand
detector = HandDetector(detectionCon=0.8, maxHands=2)

#initializing angles for movement
minHand, maxHand = 20, 250
minBar, maxBar = 400, 150
minAngle, maxAngle = 0, 180

#integrating the arduino board to python
my_port = '/dev/tty.usbmodem14101'
board = pyfirmata.Arduino('COM4')
iter8 = pyfirmata.util.Iterator(board)
iter8.start()

# pin number of our servo motor is 9
pin9 = board.get_pin('d:3:s')

#defined function to move the servo by angle as parameter
def move_servo(angle):
    pin9.write(angle)

#while loop so that it continues until escape is pressed
while True:
    #reading video from camera
    success, img = cap.read()
    hands, img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)

    if hands:
        x1, y1 = lmList[4][1], lmList[4][2]
        x2, y2 = lmList[8][1], lmList[8][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        cv2.circle(img, (x1, y1), 15, (0, 0, 255), cv2.FILLED)
        cv2.circle(img, (x2, y2), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.circle(img, (cx, cy), 15, (0, 0, 255), cv2.FILLED)

        length = math.hypot(x2 - x1, y2 - y1)
        servoVal = np.interp(length, [minHand, maxHand], [minAngle, maxAngle])
        print(int(servoVal))

        #rectangular box that determines length between index finger and thumb with relation to the computer vision trajectory
        cv2.rectangle(img, (100,100), (360, 30), (255, 0, 0), cv2.FILLED)
        cv2.putText(img, f'Length: {int(length)}', (130, 70), cv2.FONT_HERSHEY_PLAIN, 2,
                    (0, 255, 255), 3)

        # rectangular box that determines angle between index finger and thumb with relation to the computer vision trajectory and length
        cv2.rectangle(img, (500, 100), (760, 30), (0, 255, 255), cv2.FILLED)
        cv2.putText(img, f'Servo: {int(servoVal)}', (530, 70), cv2.FONT_HERSHEY_PLAIN, 2,
                    (255, 0, 0), 3)

        #vertical rectangular box showing the amount of movement triggered by angle between fingers, increases and decreases based on movement
        bar = np.interp(length, [minHand, maxHand], [minBar, maxBar])
        cv2.rectangle(img, (1180, 150), (1215, 400), (255, 0, 0), 3)
        cv2.rectangle(img, (1180, int(bar)), (1215, 400), (0, 255, 0), cv2.FILLED)

        #moving the servo motor by angle defined - servoval
        move_servo(servoVal)

    #code to show the video captured by camera
    cv2.imshow("Image", img)

    #find key pressed on keyboard
    k=cv2.waitKey(1)

    #if key clicked is 'esc', break loop and terminate
    if(k==27):
        break

#once loop is terminated, destroy the windows of opencv and release the camera
cap.release()
cv2.destroyAllWindows()