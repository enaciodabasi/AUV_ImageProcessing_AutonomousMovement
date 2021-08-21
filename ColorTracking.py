import cv2
import numpy as np
import imutils

# Red color range / can also be defined as Numpy Arrays
colorLower = (175, 20, 20)
colorUpper = (180, 255, 255)

cap = cv2.VideoCapture(0)

# Initializing x,y and radius values in order to get rid of errors in the compiler/ removing this won't change the
# outcome of the program
x = 0
y = 0
r = 0
while True:
    (success, frame) = cap.read()

    frame = imutils.resize(frame, width=600)
    # Rotating the frame makes it easier to spot our desired color | neden hiç bilmiyorum
    frame = imutils.rotate(frame, angle=180)
    # Transform the frame from BGR color space to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Marking our desired color in a mask
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Finding the contours of our object
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    # If contour is found:
    if len(contours) > 0:

        c = max(contours, key=cv2.contourArea)
        # Find the minimum enclosing circle
        ((x, y), r) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # If the circle is big enough to be out target, mark it
        if r > 10:
            cv2.circle(frame, (int(x), int(y)), int(r),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    # Print x, y and radius values:
    print("X : ")
    print(x)
    print("Y : ")
    print(y)
    print("Radius : ")
    print(r)

    '''
    Buraya motor komutları x, y ve r değerleri üzerinden eklenilecek
    '''

    cv2.imshow("Frame", frame)

    # Closing the program
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
