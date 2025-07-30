#import necessary libraries
import os
import struct
import sys

import serial
import math
import numpy as np

sys.path.append('/home/pi/TurboPi/')
import cv2
# from picamera2 import Picamera2
from time import sleep
# import RPi.GPIO as GPIO
import threading
# import HiwonderSDK.Board as Board
import time
from variables import *
port = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")

BAUDRATE = 115200
shared = {"area": 0, "p_TARGET": "", "temp_dist": 0}

# ------------------------------------------------------------{ function declarations }-------------------------------------------------------------------------

#function used to send signals to arduino to control speeds of the dc motor and the angle of the servo motor
def connect_serial():
    try:
        return serial.Serial(port, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"[Connection Failed] {e}")
        return None

def write(speed, angle):
    global arduino  # To allow reassignment on reconnect

    speed = max(0, min(255, speed))
    angle = max(0, min(145, angle))

    payload = struct.pack('<hh', speed, angle)

    try:
        time.sleep(0.05)
        if arduino and arduino.is_open:
            arduino.write(payload)
            arduino.flush()
        else:
            raise serial.SerialException("Serial port is not open.")
    except (serial.SerialException, OSError) as e:
        print(f"[Serial Error] {e} — attempting to reconnect...")
        arduino = connect_serial()
    except Exception as e:
        print(f"[Unexpected Error] {e}")

def initialize_serial():
    start = time.time()

    global arduino
    try:
        arduino = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.1)
        print(f"Serial connection established on {port}.")
    except serial.SerialException:
        print("Failed to connect.")
        print("Took", time.time() - start, "seconds")
#takes in an array of commands (dc, servo, sleep) and executes each
def multi_write(sequence):

    for action in sequence: 
        
        #delay commands
        if action < 10: 
            time.sleep(action)
        else: 
            write(action)

#function which displays the Regions of Interest on the image
def display_roi(img, ROIs, color):
    for ROI in ROIs: 
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    
    return img

#controls buzzer
 

#returns contours of a specific LAB range of a specific region of interest
def find_contours(img_lab, lab_range, ROI):
    
    #segment image to only be the ROI
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])

    #threshold image
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    
    kernel = np.ones((5, 5), np.uint8)
    
    #perform erosion and dilation
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    
    #find contours
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    return contours

#returns the area of the largest contour in a group of contours
def max_contour(contours, ROI): 
    maxArea = 0
    maxY = 0
    maxX = 0
    mCnt = 0
    
    for cnt in contours:
        
        area = cv2.contourArea(cnt)
        
        if area > 100: 
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI[0] + w // 2
            y += ROI[1] + h
            
            #if bigger contour found, update information
            if area > maxArea:
                maxArea = area
                maxY = y
                maxX = x
                mCnt = cnt

    return [maxArea, maxX, maxY, mCnt]

#function to remove overlap in black and magenta masks or add them together
def pOverlap(img_lab, ROI, add=False):
    
        lower_mask = np.array(rBlack[0])
        upper_mask = np.array(rBlack[1])
        
        #black mask
        mask = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask, upper_mask)

        lower_mask2 = np.array(rMagenta[0])
        upper_mask2 = np.array(rMagenta[1])
        
        #magenta mask
        mask2 = cv2.inRange(img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]], lower_mask2, upper_mask2)
        
        if not add: 
            #remove any overlap from the black masks 
            mask = cv2.subtract(mask, cv2.bitwise_and(mask, mask2))
        else:
            #add black and magenta to make sure car can avoid hitting parking lots
            mask = cv2.add(mask, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        
        #perform erosion
        eMask = cv2.erode(mask, kernel, iterations=1)
        
        #find contours
        contours = cv2.findContours(eMask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        return contours

#takes in a dictionary of values to print for debugging
def display_variables(variables): 

    names = list(variables.keys())

    for i in range(len(names)):
        name = names[i]
        value = variables[name]
        # Print each item on a new line
        print(f"{name}: {value}", end="\r\n")
    
    # Move the cursor up to overwrite the previous lines
    print("\033[F" * len(names), end="")

def scale_roi(roi, scale_x, scale_y, red_x=None, green_x=None):
    """يستقبل ROI بصيغته الأصلية ويعيده بعد الضرب في عامل المقياس"""
    out = []
    for i, val in enumerate(roi):
        if isinstance(val, str):                 # نعوّض مكان الكلمات 'red', 'green'
            if 'red' in val:
                raw = red_x + (-50 if '-' in val else 0) + (50 if '+' in val else 0)
            else:  # green
                raw = green_x + (-50 if '-' in val else 0) + (50 if '+' in val else 0)
        else:
            raw = val

        # محور x أم y؟
        scaled = raw * scale_x if (i % 2 == 0) else raw * scale_y
        out.append(int(round(scaled)))
    return out



def find_pillar(contours, target, p, colour): 

    global t, s, reverse, leftArea, rightArea, turnDir, maxDist
    num_p = 0
    
    for cnt in contours: 
    
        area = cv2.contourArea(cnt)
        
        #if areas are large enough for the specific colour pillar
        if (area > 150 and colour == "red") or (area > 100 and colour == "red" and tempParking) or (area > 200 and colour == "green"):
            
            #if in the time when finding the parking lot ignore green pillars with an area below 300
            # if (tempParking and colour == "green" and area < 300):
            #     continue
            
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI3[0] + w // 2 
            y += ROI3[1] + h
            
            #calculates the distance between the pillar and the bottom middle of the screen
            temp_dist = round(math.dist([x, y], [320, 480]), 0)

            #if the pillar is close enough add it to the number of pillars
            if 50 < temp_dist < minArea: #180, 390
                num_p += 1
            
            #if the pillar is too close, stop the car and reverse to give it enough space
            # if ((area > 6500 and target == redTarget) or (area > 8000 and target == greenTarget)) and ((x <= 420 and target == greenTarget) or (x >= 220 and target == redTarget)) and not tempParking and speed != 1500: #420, 220, 370, 270, 6500
            #     # #LED2(255, 255, 0)
                
            #     #move back 
            #     # multi_write([straightConst, 0.1, 1500, 0.1, reverseSpeed, 0.5, speed])
                
            #     if tempParking:
            #         # write(speed)
            #         ...
            #     #if car is supposed to stop increase the timer as we moved back
            #     if s != 0:
            #         s += 1.5
                
            #     #in the case of the car turning for a three point turn and seeing a pillar first instead of the wall, complete it as we have already switched directions
            #     if reverse == "turning" and turnDir == "right":
            #         reverse = "done"
            #         turnDir = "left"
            #         t += 1
                
            # else:
            #     #LED2(0, 0, 0)
            #     print
            #skips current pillar and moves on to the next if it comes too close to bottom of the screen
            if y > ROI3[3] - endConst or (target == greenTarget and (leftArea > maxArea or rightArea > maxArea or temp_dist > maxDist)) or (target == redTarget and (leftArea > 11500 or rightArea > 11500 or temp_dist > maxDist)): #370, 12500
                continue
            # printer( p ,temp_dist)
 

            shared["area"] = p.area
            shared["p-TARGET"] = p.target
            shared["temp_dist"] = temp_dist
            #if this pillar is closer than the current pillar replace the current pillar and update its information
            if temp_dist < p.dist:
                p.area = area
                p.dist = temp_dist
                p.y = y
                p.x = x
                p.target = target
                p.setDimentions(w, h)
    return p, num_p

def printer():
    while True:
        print(
            f"pillar area {shared['area']}  "
            f"pillar_dist {shared['p_TARGET']}   "
            f"temp_dist {shared['temp_dist']}  ",
            end="\r"
        )
        time.sleep(0.5)
# threading.Thread(target=printer, daemon=True).start()





        
