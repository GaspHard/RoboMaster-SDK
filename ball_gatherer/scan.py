from robomaster import camera
import time
import cv2
import numpy as np
import utils





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

    # Function to detect a red ball in the camera frame
    def detect_ball():
        frame = ep_camera.read_cv2_image(strategy='newest')
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # Find contours (ball detection)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > 500:  # Adjust based on your needs
                # Calculate the center of the ball
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    return cX
        return None

    # Function to center the ball in the camera's view by rotating
    def orient_robot(tolerance=detection_tolerance):
        while True:
            frame = ep_camera.read_cv2_image(strategy='newest')
            _, frame_width, _ = frame.shape
            cX = detect_ball()

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
            while detect_ball() is None:
                start_rotation()
            stop_rotation()
            while detect_ball() is not None:
                orient_robot()
    finally:
        ep_camera.stop_video_stream()