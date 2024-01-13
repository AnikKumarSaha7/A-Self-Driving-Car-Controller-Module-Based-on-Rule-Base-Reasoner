from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import math
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
GPIO.setup(12, GPIO.IN)
GPIO.setup(18, GPIO.IN)
GPIO.setwarnings(False)
import serial 
import cv2
import numpy as np
import matplotlib.pyplot as plt

camera = PiCamera()
camera.resolution = (448, 208)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(448, 208))
time.sleep(0.1)
cX =0
distance = 0
distance1 = 0
distance2 = 0
distance3 = 0
known_width = 4.5
known_width1 = 4.6
known_width2 = 3.9
known_width3 = 3.9
known_width4 = 4.5
known_width5 = 4.5
known_distance = 58
crossroad_classifier = cv2.CascadeClassifier('cascade.xml')
fr_road_classifier = cv2.CascadeClassifier('cascade6.xml')
lr_road_classifier = cv2.CascadeClassifier('cascade5.xml')
fl_road_classifier = cv2.CascadeClassifier('f-lcascade.xml')
stop_classifier = cv2.CascadeClassifier('stopcascade.xml')
person_classifier = cv2.CascadeClassifier('personcascade.xml')
redlight_classifier = cv2.CascadeClassifier('r-lightcascade.xml')

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        image = img[20:207 , 60:400]
        image1 = image[49:160 , 0:95]
       #lane_image = np.copy(image)
        face_width = 0
        gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        crossroad = crossroad_classifier.detectMultiScale(gray, 1.0485258, 6)
        fr_road = fr_road_classifier.detectMultiScale(gray, 1.0085258, 6)
        lr_road = lr_road_classifier.detectMultiScale(gray, 1.0085258, 6)
        fl_road = fl_road_classifier.detectMultiScale(gray, 1.0085258, 6)
        stop = stop_classifier.detectMultiScale(gray, 1.0085258, 6)
        person = person_classifier.detectMultiScale(gray1, 1.0085258, 6)
        
        if len(crossroad) != 0:
            for (x,y,w,h) in crossroad:
                cv2.rectangle(image1, (x,y), (x+w,y+h), (127,0,255), 2)
                face_width = w
                print('cross_road')
                focal_length = (36*known_distance)/known_width
                distance = (known_width*focal_length)/w
        if len(fr_road) != 0:
            for (x,y,w,h) in fr_road:
                cv2.rectangle(image1, (x,y), (x+w,y+h), (0,255,0), 2)
                face_width = w
                print('fr_road')
                focal_length = (36*known_distance)/known_width1
                distance1 = (known_width1*focal_length)/w
        if len(lr_road) != 0:
            for (x,y,w,h) in lr_road:
                cv2.rectangle(image1, (x,y), (x+w,y+h), (255,255,0), 2)
                face_width = w
                print('lr_road')
                focal_length = (36*known_distance)/known_width2
                distance2 = (known_width2*focal_length)/w
        if len(fl_road) != 0:
            for (x,y,w,h) in fl_road:
                cv2.rectangle(image1, (x,y), (x+w,y+h), (0,255,255), 2)
                face_width = w
                print('fl_road')
                focal_length = (36*known_distance)/known_width3
                distance3 = (known_width3*focal_length)/w
        if len(person) != 0:
            for (x,y,w,h) in person:
                cv2.rectangle(image, (x,y), (x+w,y+h), (255,255,255), 2)
                print('person')
        if len(stop) != 0:
            for (x,y,w,h) in stop:
                cv2.rectangle(image1, (x,y), (x+w,y+h), (255,0,0), 2)
                print('stop')
        print(distance)
        print(distance1)
        print(distance2)
        print(distance3)
        
        def ptransform(image):
            input = np.float32([[30, 130], [169, 130], [30, 180], [116, 180]])
            output1 = np.float32([[0,0], [200,0], [0,200], [200,200]])

            N = cv2.getPerspectiveTransform(input,output1)
            dst = cv2.warpPerspective(image,N,(200,200))
            height = dst.shape[0]
            wide = dst.shape[1]
            return dst

        p_image = ptransform(image)
        gray_image = cv2.cvtColor(p_image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray_image,127,255,0)
        M = cv2.moments(thresh)
        if M["m00"] != 0: 
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.line(p_image, (100, 200), (cX, cY), (255, 255, 255), 5)
            cv2.line(p_image, (100, 200), (100, 100), (0, 255, 0), 5)
            cv2.putText(p_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            print('CENTROID the way =%d'%(cX))      
            
        if ((distance < 50) and (distance > 0)):
           GPIO.output(7,False)
           GPIO.output(8,True)
           GPIO.output(16,False)
           print("c_Road")
           if GPIO.input(12):
               distance = 0
           if GPIO.input(18):
                        if (cX > 107):
                               GPIO.output(22,True)
                               GPIO.output(40,True)
                               
                               print("Right")
                        
                        if (cX < 93):
                               GPIO.output(22,True)
                               GPIO.output(40,False)
                               print("Left")
                        
                        if (cX <= 107 and cX >= 93):
                            GPIO.output(22,False)
                            GPIO.output(40,True)
                            print("Forward")
                            
        if ((distance1 < 50) and (distance1 > 0)):
           GPIO.output(7,False)
           GPIO.output(8,False)
           GPIO.output(16,True)
           print("f/r_Road")
           if GPIO.input(12):
               distance1 = 0
           if GPIO.input(18):
                        if (cX > 107):
                               GPIO.output(22,True)
                               GPIO.output(40,True)
                               
                               print("Right")
                        
                        if (cX < 93):
                               GPIO.output(22,True)
                               GPIO.output(40,False)
                               print("Left")
                        
                        if (cX <= 107 and cX >= 93):
                            GPIO.output(22,False)
                            GPIO.output(40,True)
                            print("Forward")
        if ((distance2 < 50) and (distance2 > 0)):
           GPIO.output(7,False)
           GPIO.output(8,True)
           GPIO.output(16,True)
           print("l/r_Road")
           if GPIO.input(12):
               distance2 = 0
           if GPIO.input(18):
                        if (cX > 107):
                               GPIO.output(22,True)
                               GPIO.output(40,True)
                               
                               print("Right")
                        
                        if (cX < 93):
                               GPIO.output(22,True)
                               GPIO.output(40,False)
                               print("Left")
                        
                        if (cX <= 107 and cX >= 93):
                            GPIO.output(22,False)
                            GPIO.output(40,True)
                            print("Forward")
        if ((distance3 < 50) and (distance3 > 0)):
           GPIO.output(7,True)
           GPIO.output(8,False)
           GPIO.output(16,True)
           print("f/l_Road")
           if GPIO.input(12):
               distance3 = 0
           if GPIO.input(18):
                        if (cX > 107):
                               GPIO.output(22,True)
                               GPIO.output(40,True)
                               
                               print("Right")
                        
                        if (cX < 93):
                               GPIO.output(22,True)
                               GPIO.output(40,False)
                               print("Left")
                        
                        if (cX <= 107 and cX >= 93):
                            GPIO.output(22,False)
                            GPIO.output(40,True)
                            print("Forward") 
        
        if(((distance == 0) or (distance >= 50)) and ((distance1 == 0) or (distance1 >= 50)) and ((distance2 == 0) or (distance2 >= 50)) and ((distance3 == 0) or (distance3 >= 50))  and (len(stop) == 0) and (len(person) == 0)):
                        if (cX > 107 and cX !=0):
                               GPIO.output(8,True)
                               GPIO.output(7,True)
                               GPIO.output(16,False)
                               print("right")
                        
                        if (cX < 93 and cX >0):
                               GPIO.output(7,True)
                               GPIO.output(8,False)
                               GPIO.output(16,False)
                               print("left")
                        
                        if (cX <= 107 and cX >= 93):
                            GPIO.output(7,False)
                            GPIO.output(8,False)
                            GPIO.output(16,False)
                            print("forward")
                            
                        if (cX == 0):
                            GPIO.output(7,True)
                            GPIO.output(8,True)
                            GPIO.output(16,True)
                            print ("Stop")
        if (len(person) != 0):
                            GPIO.output(7,True)
                            GPIO.output(8,True)
                            GPIO.output(16,True)
                            print ("Stop")
        if (len(stop) != 0):
                            GPIO.output(7,True)
                            GPIO.output(8,True)
                            GPIO.output(16,True)
                            print ("Stop")
        
        cX =0
        #print(len(lines))
        cv2.imshow('result0', image)
        cv2.imshow('result', image1)
        
        cv2.imshow('result1',p_image)
        #plt.imshow(p_image)
        
        cv2.waitKey(1) & 0xFF
        #plt.show()
        rawCapture.truncate(0)
GPIO.cleanup()
