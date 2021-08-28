import cv2
import numpy as np
import imutils
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

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

lowerColor = (22, 93, 0)
upperColor = (45, 255, 255)

camera = cv2.VideoCapture(0)

while True:

    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=600)
    frame = imutils.resize(frame, height=300)

    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None

    if len(contours) > 0:

        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    time.sleep(2)
    print("X:")
    print(x)
    print("Y:")
    print(y)
    print("Radius:")
    print(radius)

    if x == 0 and y == 0:
        # Turn around yourself / go forward

    elif radius < 100:
        # Go forward
        set_rc_channel_pwm(5, 1700)
    elif 330 < x < 360 and 250 < y < 280:
        # Go forward
        set_rc_channel_pwm(5, 1700)
    elif x < 330:
        # Sola dön
        if y < 250:
            # Go up
            set_rc_channel_pwm(3, 1700)
        elif y > 270:
            # Go down
            set_rc_channel_pwm(3, 1300)
    elif x > 370:
        # Sağa dön
        if y < 250:
            # Go up
            set_rc_channel_pwm(3, 1700)
        elif y > 270:
            # Go down
            set_rc_channel_pwm(3, 1300)


    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
