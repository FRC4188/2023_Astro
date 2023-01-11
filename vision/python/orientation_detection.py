import cv2
import numpy as np
import math

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

while(1):
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    
    _, frame = cap.read()
    
    kernel = np.ones((8,8),np.float32)/(8**2)
    dst = cv2.filter2D(frame,-1,kernel)
    
    lower_blue = np.array([0,0,151])
    upper_blue = np.array([126,111,191])
    thresh = cv2.inRange(dst, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame, frame, mask=thresh)
    
    contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        cnt = contours[0]
        #hull = cv2.convexHull(cnt)
        
        M = cv2.moments(cnt)
        if int(M['m00']) != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            res = cv2.drawContours(res,[box],0,(0,0,255),2)
            
            box = box.tolist()
            
            avgX = 0
            avgY = 0
            
            for point in box:
                avgX += point[0]/4
                avgY += point[1]/4
                
            print(math.degrees(math.atan2(avgY-cy,avgX-cx)))
            
            cv2.drawContours(res,cnt,-1,(0,255,0),5)
            cv2.drawMarker(res, (int(cx),int(cy)), (255,0,0), markerSize=25)
            cv2.drawMarker(res, (int(avgX),int(avgY)), (0,0,255), markerSize=25)

            cv2.imshow("result", res)

cap.release()
cv2.destroyAllWindows()