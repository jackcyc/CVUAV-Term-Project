
from droneCVUAV import Drone
import cv2
import numpy as np

drone = Drone('TD23')

# PID setup
drone.kp = np.array([1.08, 1.2, 1, 25])
drone.ki = np.array([0.012, 0.015, 0, 0])
drone.kd = np.array([1.16, 1.05, 1.15, 0])
err = np.zeros((4,))
errSum = np.zeros((4,))
prevErr = np.zeros((4,))

GUIDE_ID = 0
TUNNEL_ID = 3
JUMP_ID = 4
GOAL_ID = 5

""" 0. take off """
# by keyboard


""" 1. TUNNEL """
while True:
    frame = drone.background_frame_read.frame
    arucos = drone.find_arucos(frame)
    cv2.imshow('drone', frame)
    if TUNNEL_ID in arucos:
        # aim the aruco
        rvec, tvec = drone.estimatePose(arucos[TUNNEL_ID])
        # tune range of aimming
        if abs(tvec[0]) < 5 and 0 < (-1*tvec[1]):
            break
        # tune aimming PID
        err = np.array([tvec[0], 5 - tvec[1], 0, 0])
        result, errSum, prevErr = drone.PID(err, errSum, prevErr)
        drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
    else:
        # up till see aruco
        # tune upward speed
        drone.send_rc_control(0, 0, 20, 0)
    cv2.waitKey(50)

# forward 200, down 20
# debug I don't know the coord used in this func
# tune distance
print('-> tunnel aimmed!')
exit()
drone.go_xyz_speed(200, 0, -20, 100)
drone.move_right(150)
print('phase 1 clear')

""" 2. JUMP """
# moved = True
# while True:
#     frame = drone.background_frame_read.frame
#     arucos = drone.find_arucos(frame)
#     cv2.imshow('drone', frame)
#     if GUIDE_ID in arucos:
#         # follow GUIDE_ID
#         rvec, tvec = drone.estimatePose(arucos[GUIDE_ID])
#         err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
#         result, errSum, prevErr = drone.PID(err, errSum, prevErr)
#         drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
#         moved = True
#     elif JUMP_ID in arucos:
#         # aim the aruco
#         rvec, tvec = drone.estimatePose(arucos[JUMP_ID])
#         # tune range of aimming
#         if abs(tvec[0]) < 5 and abs(tvec[1]) < 5:
#             break
#         # tune aimming PID
#         err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
#         result, errSum, prevErr = drone.PID(err, errSum, prevErr)
#         drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
#         moved = True
#     else:
#         if moved:
#             drone.send_rc_control(0, 0, 0, 0)
#             moved = False
#     cv2.waitKey(50)

# print('-> jump aimmed!')
# drone.move_up(80)
# drone.move_forward(150)
# drone.move_down(90)
# print('phase 2 clear')

""" 3 & 4. guiding """
# moved = True
# while True:
#     frame = drone.background_frame_read.frame
#     arucos = drone.find_arucos(frame)
#     cv2.imshow('drone', frame)
#     if GUIDE_ID in arucos:
#         rvec, tvec = drone.estimatePose(arucos[GUIDE_ID])
#         err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 80, 0])
#         result, errSum, prevErr = drone.PID(err, errSum, prevErr)
#         drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
#         moved = True
#     elif GOAL_ID in arucos:
#         # aim the aruco
#         rvec, tvec = drone.estimatePose(arucos[GOAL_ID])
#         # tune range of aimming
#         if abs(tvec[0]) < 5 and abs(tvec[1]) < 5:
#             break
#         # tune aimming PID
#         err = np.array([tvec[0], 0 - tvec[1], tvec[2] - 50, 0])
#         result, errSum, prevErr = drone.PID(err, errSum, prevErr)
#         drone.send_rc_control(int(result[0]), int(result[2]), int(result[1]), 0)
#         moved = True
#     else:
#         if moved:
#             drone.stop()
#             moved = False
#     cv2.waitKey(50)


# edition - safe
"""
takeoff
up 50
find arucos[TUNNEL_ID]
down 50
forward 100
find arucos[GUIDE_ID]
right 150

find arucos[GUIDE_ID]
up 50
find arucos[JUMP_ID]
forward till < 90
up 80
forward 120
down 80

find arucos[GUIDE_ID]

find arucos[GOAL_ID]
land
"""

# edition - fast
"""
takeoff
up till see arucos[TUNNEL_ID]
horizontally align arucos[TUNNEL_ID]
down tvec[1] + ?(20)
curve () () 100
# alternative : forward () forward ()

follow arucos[GUIDE_ID] till see arucos[JUMP_ID]
up ?(80) forward ?(120) down ?(80)


follow arucos[GUIDE_ID] till see arucos[GOAL_ID]

v and h align arucos[GOAL_ID]
z align arucos[GOAL_ID] (50cm)
land
"""
