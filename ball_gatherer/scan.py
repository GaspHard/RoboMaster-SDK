from robomaster import camera
import time
import cv2
import numpy as np
import utils



def scan(ep_robot):
    rotation_speed = 50
    max_rotation_angle = 360  # Limit rotation to 360 degrees
    detection_tolerance = 20  # Tolerance for centering the ball
    
    # Color range for detecting the red ball in HSV
    lower_red = np.array(utils.COLOR_RANGES["red"][0])
    upper_red = np.array(utils.COLOR_RANGES["red"][1])

    # Initialize camera and chassis
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm

    # Function to detect a red ball in the camera frame
    def detect_ball(frame, lower_color, upper_color):
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
    def center_ball(cX, frame_width, tolerance=detection_tolerance):
        center_x = frame_width // 2
        error_x = cX - center_x

        # Adjust turning based on horizontal error
        while abs(error_x) > tolerance:
            frame = ep_camera.read_cv2_image(strategy='newest')
            if detect_ball(frame, lower_red, upper_red) is None:
                return False
            if error_x > 0:
                ep_chassis.drive_speed(x=0, y=0, z=10)  # Turn right
            else:
                ep_chassis.drive_speed(x=0, y=0, z=-10)  # Turn left
        ep_chassis.drive_speed(x=0, y=0, z=0)  # Stop the robot
        return True

    def scanning():
        ep_camera.start_video_stream(display=False)

        angle_turned = 0
        ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)

        try:
            while angle_turned < max_rotation_angle:
                # Capture the video frame from the camera
                frame = ep_camera.read_cv2_image(strategy='newest')
                frame_height, frame_width, _ = frame.shape

                # Detect the red ball in the frame
                cX = detect_ball(frame, lower_red, upper_red)

                if cX is not None:
                    # Center the ball in the camera view
                    if center_ball(cX, frame_width):
                        break
                    else: 
                        angle_turned = 0

                # Update the angle turned
                angle_turned += rotation_speed * 0.04  # Assuming a small time step
        finally:
            ep_camera.stop_video_stream()
            cv2.destroyAllWindows()

        return False

    # Near-field scan
    def near_field_scan():
        utils.set_arm_low(ep_arm)  # Lower the robot's arm
        scanning()

    # Far-field scan
    def far_field_scan():
        utils.set_arm_high(ep_arm)  # Raise the robot's arm
        scanning()
    
    # Start the scan process
    if near_field_scan():
        return True
    elif far_field_scan():
        return True
    else:
        return False