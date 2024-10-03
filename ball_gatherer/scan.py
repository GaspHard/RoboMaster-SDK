from robomaster import camera
import time
import cv2
import numpy as np
import utils
from utils import detect_ball

m_info = None

def on_detect_marker(marker_info):
        #x,y,w,h,number = marker_info
        print(marker_info)

def scanning(ep_robot):
    rotation_speed = 100
    detection_tolerance = 20 
    approach_tolerance = 20

    # Initialize camera and chassis
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_vision = ep_robot.vision

    ep_camera.start_video_stream(display=False)
    frame = ep_camera.read_cv2_image(strategy='newest')
    frame_height, frame_width, _ = frame.shape

    def orient_robot_on_number(tolerance, color):
        print("orienting robot")
        while True:
            center_x = frame_width // 2
            cXrel, _ = detect_number(color, orient_or_approach=True)
            cX = cXrel*frame_width
            error_x = cX - center_x
            if abs(error_x) > tolerance:
                if error_x > 0:
                    ep_chassis.drive_speed(x=0, y=0, z=20)  # Turn right
                else:
                    ep_chassis.drive_speed(x=0, y=0, z=-20)  # Turn left
            else:
                stop()
                print("orienting done")
                return True
            

    # Function to center the ball in the camera's view by rotating
    def orient_robot(tolerance, true_center_x):
        center_x = frame_width // 2
        while True:
            cX, _, _ = detect_ball(ep_camera=ep_camera)
            if cX is None:
                return False
            error_x = cX - center_x
            if abs(error_x) > tolerance:
                if error_x > 0:
                    ep_chassis.drive_speed(x=0, y=0, z=20)  # Turn right
                else:
                    ep_chassis.drive_speed(x=0, y=0, z=-20)  # Turn left
            else:
                stop()
                return True

    def stop():
        ep_chassis.drive_speed(x=0, y=0, z=0)
    
    def start_rotation():
        ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)

    def step_rotation():
        ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)
        time.sleep(0.1)
        stop()

    def go_back():
        ep_chassis.drive_speed(x=-0.2, y=0, z=0)
        time.sleep(2)
        stop()
    
    def go_back_to_center():
        ep_chassis.drive_speed(x=-0.3, y=0, z=0)
        time.sleep(3)
        stop()
    
    def advance(sleep_time):
        ep_chassis.drive_speed(x=0.1, y=0, z=0)
        time.sleep(sleep_time)
        stop()

    def approach_robot_on_number(tolerance, color):
        print("robot approaching")
        while True:
            bottom_y = frame_height
            _, cYrel = detect_number(color, orient_or_approach=True)
            cY = cYrel*frame_width
            print("cY:", cY)
            error_y = cY - bottom_y
            if abs(error_y) > tolerance:
                if error_y > 0:
                    print("forward")
                    ep_chassis.drive_speed(x=-0.1, y=0, z=0)  # go forward
                else:
                    print("backward")
                    ep_chassis.drive_speed(x=0.1, y=0, z=0)  # go backward
            else:
                stop()
                return True

    def approach_robot(tolerance, true_center_y):
        while True:
            _, cY, _ = detect_ball(ep_camera=ep_camera)

            if cY is None:
                return False
            error_y = cY - true_center_y
            if abs(error_y) > tolerance:
                if error_y > 0:
                    ep_chassis.drive_speed(x=-0.1, y=0, z=0)  # go forward
                else:
                    ep_chassis.drive_speed(x=0.1, y=0, z=0)  # go backward
            else:
                stop()
                return True

    def on_detect_marker(marker_info):
        #x,y,w,h,number = marker_info
        #print("marker info:", marker_info)
        global m_info
        m_info = marker_info

    def detect_number(color, orient_or_approach=False):
        global m_info
        print("detect_number")
        color_dict = {
            "red": 1,
            "yellow": 2,
            "green": 3,
            "blue": 4
        }
        number = color_dict[color]
        print("ball color number:", number)
        while True:
            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            if not orient_or_approach:
                step_rotation()
            ep_vision.unsub_detect_info(name="marker")
            print("m_info:", m_info)
            if m_info:
                n = len(m_info)
                for i in range(0, n):
                    x, y, w, h, info = m_info[i]
                    if int(info) == number:
                        print(x,y)
                        return (x,y)
                    
                    




    
    
    # Start the scan and appraoch process
    try:
        true_first_image = cv2.imread("./ball_gatherer/first_approach.png")
        true_first_image = cv2.resize(true_first_image, (frame_width, frame_height))
        true_final_image = cv2.imread("./ball_gatherer/final_approach.png")
        true_final_image = cv2.resize(true_final_image, (frame_width, frame_height))
        cX_first_true, cY_first_true, _ = detect_ball(ep_camera=None, frame=true_first_image)
        cX_final_true, cY_final_true, _ = detect_ball(ep_camera=None, frame=true_final_image)
        while True:
            m_info = None
            utils.set_arm_low(ep_arm)  # Lower the robot's arm
            utils.open_gripper(ep_gripper)
            while detect_ball(ep_camera=ep_camera)[0] is None:
                start_rotation()
            stop()
            while detect_ball(ep_camera=ep_camera)[0] is not None:
                cX_cam, cY_cam, _ = detect_ball(ep_camera=ep_camera)
                if cX_cam is None or cY_cam is None:
                    break
                cX_first_error = cX_first_true - cX_cam
                cY_first_error = cY_first_true - cY_cam
                if abs(cX_first_error) > approach_tolerance:
                    if not orient_robot(approach_tolerance, cX_first_true):
                        break
                if abs(cY_first_error) > approach_tolerance:
                    if not approach_robot(approach_tolerance, cY_first_true):
                        break
                if abs(cX_first_error) < approach_tolerance and abs(cY_first_error) < approach_tolerance :
                    stop()
                    utils.set_arm_to_grab(ep_arm)
                    cX_cam, cY_cam, _ = detect_ball(ep_camera=ep_camera)
                    if cX_cam is None or cY_cam is None:
                        break
                    cX_final_error = cX_final_true - cX_cam
                    cY_final_error = cY_final_true - cY_cam
                    if abs(cX_final_error) > approach_tolerance:
                        if not orient_robot(approach_tolerance, cX_final_true):
                            break
                    if abs(cY_final_error) > approach_tolerance:
                        if not approach_robot(approach_tolerance, cY_final_true):
                            break
                    if abs(cX_final_error) < approach_tolerance and abs(cY_final_error) < approach_tolerance :
                        stop()
                        color = utils.grab_ball(ep_arm, ep_gripper, ep_camera)
                        if color is None:
                            break
                        else:
                            utils.set_arm_calib(ep_arm)
                            go_back()
                            cXrel, cYrel = detect_number(color)
                            cX = cXrel * frame_width
                            #cY = cYrel * frame_height
                            #print(cX, cY)
                            orient_robot_on_number(approach_tolerance, color)
                            approach_robot_on_number(approach_tolerance, color)
                            utils.set_arm_to_store(ep_arm)
                            stop()
                            advance(3.5)
                            utils.open_gripper(ep_gripper)
                            go_back_to_center()
                            break

    finally:
        ep_camera.stop_video_stream()
        ep_vision.unsub_detect_info(name="marker")