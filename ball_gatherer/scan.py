from robomaster import camera
import time
import cv2
import numpy as np
import utils
from utils import detect_ball




def scanning(ep_robot):
    rotation_speed = 100
    detection_tolerance = 20  # Tolerance for centering the ball
    
    
    # Color range for detecting the red ball in HSV
    lower_color = np.array(utils.COLOR_RANGES["red"][0])
    upper_color = np.array(utils.COLOR_RANGES["red"][1])

    # Initialize camera and chassis
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm

    ep_camera.start_video_stream(display=False)

    # Function to center the ball in the camera's view by rotating
    def orient_robot(tolerance=detection_tolerance):
        while True:
            frame = ep_camera.read_cv2_image(strategy='newest')
            _, frame_width, _ = frame.shape
            cX, color = detect_ball(ep_camera=ep_camera)

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
                stop_rotation()
                return True

    def stop_rotation():
        ep_chassis.drive_speed(x=0, y=0, z=0)
    
    def start_rotation():
        ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)
    
    # Start the scan process
    try:
        while True:
            utils.set_arm_low(ep_arm)  # Lower the robot's arm
            while detect_ball(ep_camera=ep_camera)[0] is None:
                start_rotation()
            stop_rotation()
            while detect_ball(ep_camera=ep_camera)[0] is not None:
                orient_robot()
    finally:
        ep_camera.stop_video_stream()