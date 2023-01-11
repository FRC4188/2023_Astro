import cv2
import numpy as np
import math

# Define constants.
blur_size = 8
lower_rgb = np.array([0,0,151])
upper_rgb = np.array([126,111,191])

# Create a video capture object to retrieve frames from the camera.
cap = cv2.VideoCapture(0)

# Main loop repeats until esc key is pressed; on the roborio there will be no while loop, vision code will be triggered by the robot main loop.
while(1):
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    
    _, frame = cap.read()
    
    # Define a box-blur kernal to use as a blurring template and apply it to the frame.
    kernel = np.ones((blur_size,blur_size),np.float32)/(blur_size**2)
    dst = cv2.filter2D(frame,-1,kernel)
    
    # Isolate the pixels within the rgb threshold and draw a contour around them.
    thresh = cv2.inRange(dst, lower_rgb, upper_rgb)
    res = cv2.bitwise_and(frame, frame, mask=thresh)
    contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # Only proceed if the list of contours is adequate.
    if len(contours) > 0:
        cnt = contours[0]
        #hull = cv2.convexHull(cnt)
        
        # Only proceed if the list of moments is adequate.
        M = cv2.moments(cnt)
        if int(M['m00']) != 0:
            
            # Determine the center of mass of the chosen contour.
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # Find the minimum area rectange which confines the contour.
            # I'm considering changing this to the minimum area circle to eliminate a fringe-case error but it's the same idea.
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            res = cv2.drawContours(res,[box],0,(0,0,255),2)
            box = box.tolist()
            
            # Find the center of the rectangle.
            avgX = 0
            avgY = 0
            for point in box:
                avgX += point[0]/4
                avgY += point[1]/4
            
            # The angle of the cone is the angle from the center of mass to the center of the rectangle.
            # Another way to think of it is the center of mass is weighed from the center of the rectangle towards the bottom of
            #   the cone by the wider base, so the line from the center of mass to the center of the rectangle points in the same direction as the cone.
            print(math.degrees(math.atan2(avgY-cy,avgX-cx)))
            
            # Show the marks of the center of mass and center of the rectangle on the thresholded image to visualize the algorithm.
            cv2.drawContours(res,cnt,-1,(0,255,0),5)
            cv2.drawMarker(res, (int(cx),int(cy)), (255,0,0), markerSize=25)
            cv2.drawMarker(res, (int(avgX),int(avgY)), (0,0,255), markerSize=25)

            cv2.imshow("result", res)

cap.release()
cv2.destroyAllWindows()