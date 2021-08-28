import cv2
import numpy as np
import imutils

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


colorLower = (22, 93, 0)
colorUpper = (45, 255, 255)


camera = cv2.VideoCapture(0)


while True:

    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=640)
    frame = imutils.resize(frame, height=480)

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
            print("X:")
            print(x)
            print("Y:")
            print(y)

        

    cv2.imshow("Frame", frame)




    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
