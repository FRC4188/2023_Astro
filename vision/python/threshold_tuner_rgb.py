import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

img = np.zeros((len(frame),len(frame[0]),3), np.uint8)
cv2.namedWindow('image')

cv2.createTrackbar('Rm','image',0,255,nothing)
cv2.createTrackbar('Gm','image',0,255,nothing)
cv2.createTrackbar('Bm','image',0,255,nothing)
cv2.createTrackbar('RM','image',0,255,nothing)
cv2.createTrackbar('GM','image',0,255,nothing)
cv2.createTrackbar('BM','image',0,255,nothing)

switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    rm = cv2.getTrackbarPos('Rm','image')
    gm = cv2.getTrackbarPos('Gm','image')
    bm = cv2.getTrackbarPos('Bm','image')
    rM = cv2.getTrackbarPos('RM','image')
    gM = cv2.getTrackbarPos('GM','image')
    bM = cv2.getTrackbarPos('BM','image')
    state = cv2.getTrackbarPos(switch,'image')
    
    _, frame = cap.read()

    if state == 0:
        img = frame
    else:
        lower_blue = np.array([bm,gm,rm])
        upper_blue = np.array([bM,gM,rM])
        mask = cv2.inRange(frame, lower_blue, upper_blue)
        img = mask

cap.release()
cv2.destroyAllWindows()