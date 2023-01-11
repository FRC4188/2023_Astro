import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# Create a black image, a window
img = np.zeros((len(frame),len(frame[0]),3), np.uint8)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Hm','image',0,255,nothing)
cv2.createTrackbar('Sm','image',0,255,nothing)
cv2.createTrackbar('Vm','image',0,255,nothing)
cv2.createTrackbar('HM','image',0,255,nothing)
cv2.createTrackbar('SM','image',0,255,nothing)
cv2.createTrackbar('VM','image',0,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of four trackbars
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
        kernel = np.ones((10,10),np.float32)/(10**2)
        dst = cv2.filter2D(frame,-1,kernel)
        img = dst
    else:
        lower_blue = np.array([hm,sm,vm])
        upper_blue = np.array([hM,sM,vM])
        kernel = np.ones((10,10),np.float32)/(10**2)
        dst = cv2.filter2D(frame,-1,kernel)
        mask = cv2.inRange(dst, lower_blue, upper_blue)
        img = mask

cap.release()
cv2.destroyAllWindows()