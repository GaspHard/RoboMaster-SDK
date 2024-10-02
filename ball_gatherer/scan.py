from robomaster import camera
import time
import cv2
import numpy as np
import utils
from utils import detect_ball




def scanning(ep_robot):
    rotation_speed = 100
    detection_tolerance = 20 
    approach_tolerance = 20

    # Initialize camera and chassis
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

    ep_camera.start_video_stream(display=False)
    frame = ep_camera.read_cv2_image(strategy='newest')
    frame_height, frame_width, _ = frame.shape

    # Function to center the ball in the camera's view by rotating
    def orient_robot(tolerance, true_center_x):
        while True:
            cX, _, _ = detect_ball(ep_camera=ep_camera)

            center_x = frame_width // 2
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
    
    # Start the scan and appraoch process
    try:
        true_first_image = cv2.imread("./ball_gatherer/first_approach.png")
        true_first_image = cv2.resize(true_first_image, (frame_width, frame_height))
        true_final_image = cv2.imread("./ball_gatherer/final_approach.png")
        true_final_image = cv2.resize(true_final_image, (frame_width, frame_height))
        cX_first_true, cY_first_true, _ = detect_ball(ep_camera=None, frame=true_first_image)
        cX_final_true, cY_final_true, _ = detect_ball(ep_camera=None, frame=true_final_image)
        while True:
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
                        return True   
    finally:
        ep_camera.stop_video_stream()