
from droneCVUAV import Drone
import cv2
import numpy as np
import time

drone = Drone('G1')

# PID setup

# G1
# drone.kp = np.array([1, 1.05, 0.6, 10])
# drone.ki = np.array([0.008, 0.015, 0, 0])
# drone.kd = np.array([1.16, 1.15, 1.05, 0])

drone.kp = np.array([0.8, 1.2, 0.8, 25])
drone.ki = np.array([0.012, 0.02, 0, 0])
drone.kd = np.array([1.2, 1.05, 1.15, 0])

err = np.zeros((4,))
errSum = np.zeros((4,))
prevErr = np.zeros((4,))

GUIDE_ID = 0
TUNNEL_ID = 3
JUMP_ID = 4
GOAL_ID = 5
TURN_ID = 9

""" 0. take off """
# by keyboard
# while not drone.is_flying:
#     time.sleep(0.1)

drone.takeoff()

""" 1. TUNNEL """
while True:
    frame = drone.background_frame_read.frame
    arucos = drone.find_arucos(frame)
    if TUNNEL_ID in arucos:
        rvec, tvec = drone.estimatePose(arucos[TUNNEL_ID])
        forward_distance = tvec[2] - 40
        break
    cv2.imshow('drone', frame)
    cv2.waitKey(50)

drone.move_forward(160)
drone.move_down(60)
drone.move_forward(130)
drone.go_xyz_speed(-30, -200, 50, 100)
print('phase 1 clear')
# drone.land()
# exit()

""" 2. JUMP """
moved = True
while True:
    frame = drone.background_frame_read.frame
    arucos = drone.find_arucos(frame)
    if GUIDE_ID in arucos:
        # follow GUIDE_ID
        rvec, tvec = drone.estimatePose(arucos[GUIDE_ID])
        err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
        result, errSum, prevErr = drone.PID(err, errSum, prevErr)
        drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
        moved = True
    elif JUMP_ID in arucos:
        # aim the aruco
        rvec, tvec = drone.estimatePose(arucos[JUMP_ID])
        # tune range of aimming
        if abs(tvec[0]) < 10 and abs(tvec[1]) < 10 and (tvec[2] < 90):
            drone.stop()
            break
        err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
        result, errSum, prevErr = drone.PID(err, errSum, prevErr)
        drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
        moved = True
    else:
        if moved:
            drone.send_rc_control(0, 0, 0, 0)
            moved = False
    cv2.imshow('drone', frame)
    cv2.waitKey(50)

print('-> jump aimmed!')
drone.move_up(50)
drone.move_forward(150)
drone.move_down(90)
print('phase 2 clear')

""" 3 & 4. guiding """

moved = True
turned = False
while True:
    frame = drone.background_frame_read.frame
    arucos = drone.find_arucos(frame)
    if GUIDE_ID in arucos:
        rvec, tvec = drone.estimatePose(arucos[GUIDE_ID])
        err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
        result, errSum, prevErr = drone.PID(err, errSum, prevErr)
        drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
        moved = True
    elif turned == False and TURN_ID in arucos:
        drone.rotate_clockwise(45)
        turned = True
    elif GOAL_ID in arucos:
        # drone.kp = np.array([0.8, 1.2, 0.4, 15])
        # drone.kd = np.array([0.9, 0.9, 0.6, 0])
        # aim the aruco
        rvec, tvec = drone.estimatePose(arucos[GOAL_ID])
        # tune range of aimming
        if abs(tvec[0]) < 7 and (tvec[2] < 65):
            drone.stop()
            break
        # tune aimming PID
        err = np.array([0.8*tvec[0], 0.8*(0 - tvec[1]), 0.8(tvec[2] - 60), 0])
        result, errSum, prevErr = drone.PID(err, errSum, prevErr)
        drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
        moved = True
    else:
        if moved:
            drone.send_rc_control(0, 0, 0, 0)
            moved = False
    cv2.imshow('drone', frame)
    cv2.waitKey(50)

print('end')
drone.land()
