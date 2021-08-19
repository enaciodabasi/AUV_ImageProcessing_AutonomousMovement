import cv2
import numpy as np
import time
from pymavlink import mavutil

capture = cv2.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 400)

# Establishing connection to the AUV
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)


# User- defined functions for control of the motors via PWM values
def armAuv():
    master.mav.command_long_send(
        master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )


def disarmAuv():
    master.mav.command_long_send(
        master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )


def set_rc_channel_pwm(channel_id, pwm=1500):
    if channel_id < 1 or channel_id > 11:
        print("Channel doesn't exist")
        return

    rc_channel_values = [1500 for _ in range(8)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *rc_channel_values
    )


def resetPWMs():
    for i in range(1, 6):
        set_rc_channel_pwm(i, 1500)


disarmAuv()
time.sleep(2)
armAuv()
resetPWMs()

# Detecting x and y coordinates and radius of detected circles with OpenCV
while True:
    # Capturing the video feed
    success, img = capture.read()
    output = img.copy()

    # Converting the captured frame to Grayscale color-space for the implementation of the HoughCircles algorithm
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Smoothing the image via medianBlur in order to get more accurate circles
    gray_img = cv2.medianBlur(gray_img, 5)
    # Applying HoughCircles and storing the x, y and r values in a Numpy Array
    circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 20, minRadius=0, maxRadius=1000)

    # If a circle is found, mark it on the output screen and control the AUV via ArduSub and PyMavlink
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)

    cv2.imshow("Circle Detection", output)

    # Code to control various motors connected to the AUV via their PMW values

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cv2.destroyAllWindows()
