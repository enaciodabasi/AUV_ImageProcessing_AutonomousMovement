import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lowerColor = (22, 93, 0)
upperColor = (45, 255, 255)

while True:

    success, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame, lowerColor, upperColor)
    output = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Color Detection", output)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
