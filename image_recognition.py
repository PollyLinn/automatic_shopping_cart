#import all the required modules
import numpy as np
import serial
import time
import sys
import cv2

# 視窗長寬，還不能調
width = 500
height = 400
global xxx  # 用來記錄error，可以傳serial。 # 不知道是不是一定要這樣設
r = 50

#Setup Communication path for arduino
# 連接arduino的 port & 包率
arduino = serial.Serial('COM10', 115200)
time.sleep(2)  # 程式暫停幾秒，睡眠
print("Connected to arduino...")
#importing the Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier('facedetect.xml')
#To capture the video stream from webcam.
cap = cv2.VideoCapture(1)
#Read the captured image, convert it to Gray image and find faces
while 1:
    ret, img = cap.read()
    if ret:
        cv2.imshow('img',img)
    # cv2.namedWindow('img', 0)
        cv2.resizeWindow('img',width,height)
        cv2.line(img,(width,int(height/2)),(0,int(height/2)),(0,255,0),1)
        cv2.line(img,(int(width/2),0),(int(width/2),height),(0,255,0),1)
        cv2.circle(img, (int(width/2), int(height/2)), 5, (255, 255, 255), -1)
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3)
        # 邊長2r的正方形，放在正中間
        cv2.rectangle(img,(int(width/2-r),int(height/2-r)),(int(width/2+r),int(height/2+r)),(0,255,0),5)
#detect the face and make a rectangle around it.
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),5) #(,左上點,右下點,顏色,粗度)
        roi_gray  = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        arr = {y:y+h, x:x+w}
# CALCULATE
        # 人x的中心位置
        xx = int(x+(x+h))/2
        yy = int(y+(y+w))/2
        # ey < 0 人近 ; ey > 0 人小 人比較遠
        ey = 2*r - h
        e = int(xx - int(width / 2)) # e > 0，右轉
        # 當差很小，就不左右動
        # if abs(e)<20:
        #     xxx = 0
        # else:
        xxx = e/10

# Center of roi (Rectangle)
        # turn right x >0
        print ('X:')
        print (xxx)
        # print('Y:')
        # print (yy)
        center = (xx,yy)

# sending data to arduino
        # data 的 x 傳 左右轉 ;y 傳 前後動
        data = "X{0}Y{1}".format(int(xxx), int(ey)) #Converting to an int to
        bdata = b'data'                            #remove the ".0" from each value
        print(data)
        arr = bytes(data, 'ascii')
        # 兩個板先傳同一個值
        arduino.write(arr)
#Display the stream.
    cv2.imshow('img',img)
#Hit 'Esc' to terminate execution
    k = cv2.waitKey(30) & 0xff
    if k == 27:
       break