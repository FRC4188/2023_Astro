import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

img = np.zeros((len(frame),len(frame[0]),3), np.uint8)
cv2.namedWindow('image')

cv2.createTrackbar('Hm','image',0,255,nothing)
cv2.createTrackbar('Sm','image',0,255,nothing)
cv2.createTrackbar('Vm','image',0,255,nothing)
cv2.createTrackbar('HM','image',0,255,nothing)
cv2.createTrackbar('SM','image',0,255,nothing)
cv2.createTrackbar('VM','image',0,255,nothing)

switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    hm = cv2.getTrackbarPos('Hm','image')
    sm = cv2.getTrackbarPos('Sm','image')
    vm = cv2.getTrackbarPos('Vm','image')
    hM = cv2.getTrackbarPos('HM','image')
    sM = cv2.getTrackbarPos('SM','image')
    vM = cv2.getTrackbarPos('VM','image')
    state = cv2.getTrackbarPos(switch,'image')
    
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if state == 0:
        img = frame
    else:
        lower_blue = np.array([hm,sm,vm])
        upper_blue = np.array([hM,sM,vM])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        frame = cv2.bitwise_and(frame, frame, mask= mask)
        img = frame

cap.release()
cv2.destroyAllWindows()