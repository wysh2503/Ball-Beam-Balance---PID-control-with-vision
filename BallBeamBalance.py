# Importing dependencies
import cv2
import numpy as np
import time
import serial

# Setting arduino board com port and baudrate
ArduinoSerial = serial.Serial('com3', 115200)
time.sleep(2)

def nothing(): pass

cv2.namedWindow('TargetPosition')
cv2.createTrackbar('setpoint','TargetPosition', 320, 640, nothing)

cv2.namedWindow('Trackbars')
cv2.createTrackbar('HueLow','Trackbars',159,179,nothing)
cv2.createTrackbar('HueHigh','Trackbars',179,179,nothing)
cv2.createTrackbar('SatLow','Trackbars',153,255,nothing)
cv2.createTrackbar('SatHigh','Trackbars',255,255,nothing)
cv2.createTrackbar('ValLow','Trackbars',122,255,nothing)
cv2.createTrackbar('ValHigh','Trackbars',255,255,nothing)

cv2.namedWindow('P_I_D')
cv2.createTrackbar('kp','P_I_D', 30, 150, nothing)
cv2.createTrackbar('ki','P_I_D', 0, 20, nothing)
cv2.createTrackbar('kd','P_I_D', 20, 100, nothing)

servoNeutralPos = 91

cam = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# PID initialisation
# kp = 0.20
# ki = 0
# kd = 0.025
previous_error = 0
currentTime = 0
pid_i = 0

while True:
    success, img = cam.read()

    setpoint = cv2.getTrackbarPos('setpoint', 'TargetPosition')

    hueLow = cv2.getTrackbarPos('HueLow', 'Trackbars')
    hueHigh = cv2.getTrackbarPos('HueHigh', 'Trackbars')
    satLow = cv2.getTrackbarPos('SatLow', 'Trackbars')
    satHigh = cv2.getTrackbarPos('SatHigh', 'Trackbars')
    valLow = cv2.getTrackbarPos('ValLow', 'Trackbars')
    valHigh = cv2.getTrackbarPos('ValHigh', 'Trackbars')

    kp = cv2.getTrackbarPos('kp','P_I_D')/100.
    ki = cv2.getTrackbarPos('ki','P_I_D')/1000.
    kd = cv2.getTrackbarPos('kd','P_I_D')/1000.

    # GaussianBlur,Converting image to HSV format
    # imgblurred = cv2.GaussianBlur(img, (11, 11), 0)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Masking orange color
    mask = cv2.inRange(imgHSV, (hueLow, satLow, valLow), (hueHigh, satHigh, valHigh))
    # Removing Noise from the mask
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)
    # Extracting contour
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)
    # Drawing Contour
    cv2.drawContours(img, contours, -1, (255, 0, 0), 3)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        (x, y, w, h) = cv2.boundingRect(cnt)
        if area > 100:
            #cv2.drawContours(img,[cnt],0,(255,0,0),3)
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
            centerX = x + w/2
            centerY = y + h/2
            # draw the max area contour and center of the shape on the image
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 2)
            cv2.circle(img, (int(centerX), int(centerY)), 7, (255, 255, 255), -1)
            cv2.putText(img, "center", (int(centerX - 20), int(centerY - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Drawing a vertical line at the centre with Blue color
            cv2.line(img, (int(centerX), 0), (int(centerX), int(height)), (255, 0, 0), 2)

            # PID calculation
            error = (centerX - setpoint)
            # Proportional
            pid_p = kp * error

            try:
                # Integral
                # if -15 < error < 15:
                #     pid_i = pid_i + (ki * error)
                # Derivative
                previousTime = currentTime
                currentTime = time.time()
                elapsedTime = currentTime - previousTime
                pid_d = kd * ((error - previous_error) / elapsedTime)
                if -40 < error < 40:
                    pid_i = pid_i + (ki * elapsedTime)

                PID = pid_p + pid_i + pid_d

                previous_error = error
                servoPos = servoNeutralPos - PID

                if servoPos < 60:
                    servoPos = 60
                if servoPos > 130:
                    servoPos = 130
                servoPos = int(servoPos)
                # ArduinoSerial.write(str(chr(int(servoPos))))
                ArduinoSerial.write([servoPos])
                print(f'servoPos {servoPos}')
                print(f'error {error}')
            except:
                pass
        break
    # Drawing a vertical central line at middle of frame
    cv2.line(img, (int(setpoint), 0), (int(setpoint), int(height)), (0, 0, 255), 2)

    cv2.imshow("mask", mask)
    cv2.imshow("webcam", img)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cam.release()
cv2.destroyAllWindows()
