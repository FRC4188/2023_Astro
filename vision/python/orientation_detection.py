import cv2
import numpy as np
import math

# Define constants.
blur_size = 10
lower_bgr = np.array([24,64,84])
upper_bgr = np.array([161,255,255])

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
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Isolate the pixels within the rgb threshold.
    thresh = cv2.inRange(frame, lower_bgr, upper_bgr)
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
    
    # The erosion helps eliminate "speckels".
    kernel = np.ones((5, 5), np.uint8)
    img = cv2.erode(thresh, kernel)
    
    # Draw contours around the areas which made it through the threshold.
    res = cv2.bitwise_and(frame, frame, mask=thresh)
    contours, _ = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    
    # Only proceed if the list of contours is adequate.
    if len(contours) > 0:
        
        # Isolate the contour which has the greatest area.
        cnt_index = 0
        cnt_area = 0
        for i in range(len(contours)):
            a = cv2.moments(contours[i])['m00']
            if a > cnt_area:
                cnt_area = a
                cnt_index = i
        
        cnt = contours[cnt_index]
        
        # Determine the triangle which best fits the cone.
        tri = cv2.minEnclosingTriangle(cnt)[1]
        tri = np.intp(tri)
        res = cv2.drawContours(res,[tri],0,(0,0,255),2)
        tri = tri.tolist()
        
        # Determine the average distance to each other corner for each corner.
        avg_dists = [0, 0, 0]
        for i in range(len(tri)):
            dist = 0
            for j in range(len(tri)):
                if not j == i:
                    dist += math.hypot(tri[i][0][0] - tri[j][0][0], tri[i][0][1] - tri[j][0][1]) / 2
            avg_dists[i] = dist
        
        # Determine the average difference between average distances.
        avg_diffs = [0, 0, 0]
        for i in range(len(avg_dists)):
            diff = 0
            for j in range(len(avg_dists)):
                if not i == j:
                    diff += abs(avg_dists[i]-avg_dists[j])/2
            avg_diffs[i] = diff
        
        # Determine which corner is the top of the cone by how different its avererage distance is from the other corners.
        max_diff = 0
        max_index = 0
        for i in range(len(avg_diffs)):
            if avg_diffs[i] > max_diff:
                max_diff = avg_diffs[i]
                max_index = i
        
        # Isolate the positions of the base corners of the triangle.
        base = []
        for i in range(len(tri)):
            if not i == max_index:
                base.append(tri[i])

        # Find the position of the center of the base.
        bx = base[0][0][0]/2 + base[1][0][0]/2
        by = base[0][0][1]/2 + base[1][0][1]/2
        
        # Calculate the angle the triangle is facing based on the center of the base and the top corner.
        angle = (math.degrees(math.atan2(bx - tri[max_index][0][0], by - tri[max_index][0][1])) + 180) % 360 - 180
        
        # Draw the detected features on the image to visualize the algorithm.
        base = np.intp(base)
        frame = cv2.drawContours(res, [base], 0, (255, 0, 0), 3)
        cv2.drawMarker(frame, (int(bx), int(by)), (255, 0, 0), markerSize=25)
        point = (tri[max_index][0][0],tri[max_index][0][1])
        cv2.drawMarker(frame, point, (0,0,255), markerSize=25)
        cv2.drawContours(frame,contours,-1,(0,255,0),2)
        cv2.putText(frame, "{angle: .1f} degrees".format(angle= angle), point, cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,cv2.LINE_AA)
    cv2.imshow("result", frame) 

cap.release()
cv2.destroyAllWindows()