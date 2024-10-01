from robomaster import robot, camera, config
import socket
import cv2
import time
import numpy as np

COLOR_RANGES = {
    "blue": ([95, 150, 150], [110, 255, 255]),   # Blue HSV range
    "green": ([55, 150, 150], [65, 255, 255]),   # Green HSV range
    "red": ([0, 150, 150], [10, 255, 255]),      # Red HSV range
    "yellow": ([25, 150, 150], [35, 255, 255])} # Yellow HSV range

def get_ip_starting_with(prefix):
    """
    Get the local IP address that starts with the given prefix.
    
    Args:
    prefix (str): The prefix to filter the IP addresses by (e.g., '192.168.5.')
    
    Returns:
    str: The first local IP address that starts with the prefix, or None if not found.
    """
    hostname = socket.gethostname()
    addresses = socket.getaddrinfo(hostname, None)
    
    for address in addresses:
        ip = address[4][0]
        if ip.startswith(prefix):
            return ip
    
    return None  # Return None if no matching IP address is found

def set_arm_low(arm):
    arm.moveto(x=150, y=60).wait_for_completed()

def set_arm_high(arm):
    arm.moveto(x=110, y=100).wait_for_completed()

def set_arm_to_grab(arm):
    arm.moveto(x=180, y=0).wait_for_completed()

def camera_control(ep_robot):
    print("Starting camera preview...")
    cam = ep_robot.camera
    arm = ep_robot.robotic_arm
    gripper = ep_robot.gripper

    # Start streaming from the camera
    cam.start_video_stream(display=False, resolution=camera.STREAM_540P)

    while True:
        # Capture frame from the camera
        frame = cam.read_cv2_image(strategy='newest', timeout=1)

        if frame is not None:
            # Display the frame in a window
            cv2.imshow("RoboMaster EP Camera", frame)

        # Check for key presses every 1ms
        key = cv2.waitKey(1) & 0xFF

        # Exit the preview when 'q' is pressed
        if key == ord('q'):
            break
        
        # Lower the arm when 'l' is pressed
        if key == ord('l'):
            print("Lowering the arm...")
            set_arm_low(arm)  # Call the function to lower the arm

        # Lower the arm when 'l' is pressed
        if key == ord('h'):
            print("Raising the arm...")
            set_arm_high(arm)  # Call the function to lower the arm
        
        if key == ord('o'):
            print("Opening gripper...")
            open_gripper(gripper=gripper)  # Call the function to lower the arm

        if key == ord('g'):
            print("gripping...")
            grab_ball(arm=arm, gripper=gripper)  # Call the function to lower the arm

            # Step 5: Check the camera to see if the ball is still visible
            for _ in range(5):  # Check a few frames to confirm
                frame = cam.read_cv2_image(strategy='newest', timeout=1,)
                frame = cv2.resize(frame, (960, 540))

                if frame is not None:
                    lower_color = (0, 100, 100)   # Lower HSV values for red
                    upper_color = (10, 255, 255)  # Upper HSV values for red
                    color_range = (lower_color, upper_color)

                    # Use color detection to see if the ball is still in the frame 
                    #print(detect_ball(frame=frame, crop = True, x=456, y=378, w=586, h=347)[0]) # x=456, y=378, w=586, h=347
                    if detect_ball(frame=frame, crop = False, x=456, y=378, w=586, h=347, area_threshold=200)[0] is not None:
                        print("Ball still visible in camera feed.")
                        print("BALL FOUND")  # Ball is still visible, grab likely failed
                time.sleep(0.5)  # Small delay between checks

    # Stop the video stream and close OpenCV window
    cam.stop_video_stream()
    cv2.destroyAllWindows()


def grab_ball(arm, gripper):
    """
    Grabs a ball using the gripper, and checks if the ball is successfully grabbed using resistance feedback or the camera.
    
    Args:
    arm (robomaster.robotic_arm): The robotic arm object to control the gripper arm.
    gripper (robomaster.gripper): The gripper object to grab the ball.
    cam (robomaster.camera): The camera object to verify if the ball is grabbed.
    
    Returns:
    bool: True if the ball is successfully grabbed, False otherwise.
    """
    # Step 2: Open the gripper before trying to grab
    print("Opening the gripper...")
    gripper.open(power=50)  # Adjust power as necessary
    time.sleep(1)

    # Step 3: Move the arm down to grab the ball
    print("Moving arm to grab position...")
    set_arm_to_grab(arm)  # Adjust these coordinates as needed
    time.sleep(1)

    # Step 4: Close the gripper to grab the ball
    print("Closing the gripper to grab the ball...")
    gripper.close(power=50)  # Close the gripper with 50% power
    time.sleep(1)

def open_gripper(gripper):
    print("Opening the gripper...")
    gripper.open(power=50)  # Adjust power as necessary

def detect_ball_in_roi(frame, color_range, x=456, y=378, w=586, h=347):
    """
    Detect if the ball is inside a specific region of interest (ROI) based on color.

    Args:
    frame (np.ndarray): The frame from the camera feed.
    color_range (tuple): The lower and upper HSV color range for the ball.
    x, y, w, h (int): Coordinates and dimensions of the region of interest (ROI).

    Returns:
    bool: True if the ball is detected in the ROI, False otherwise.
    """
    # Crop the frame to the specified ROI
    roi_frame = frame[y:y+h, x:x+w]

    # Convert the cropped ROI to the HSV color space
    hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

    # Apply the color mask to detect the ball in the ROI
    lower_color, upper_color = color_range
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Check if there's enough pixels of the specified color in the ROI
    ball_detected = cv2.countNonZero(mask) > 200  # Adjust the threshold as needed

    return ball_detected


def select_roi_from_image(image_path=".\\ball_gatherer\\ball_grabbed.png"):
    """
    Allows the user to select a Region of Interest (ROI) in an image and returns the coordinates.

    Args:
    image_path (str): The path to the image file.

    Returns:
    tuple: The (x, y, w, h) coordinates of the selected region.
    """
    # Load the image
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not read the image from {image_path}")
        return None

    # Select the ROI interactively
    roi = cv2.selectROI("Select ROI", image, showCrosshair=True, fromCenter=False)

    # Close the window
    cv2.destroyAllWindows()

    # Extract ROI coordinates
    x, y, w, h = roi

    print(f"Selected ROI: x={x}, y={y}, w={w}, h={h}")
    return roi


# Function to detect a red ball in the camera frame
def detect_ball(ep_camera = None, frame = None, area_threshold = 500, crop = False, x = 0, y = 0, h = 0, w = 0):
    if frame is None:
    # Read the latest frame from the camera
        frame = ep_camera.read_cv2_image(strategy='newest')

    # Convert the frame to HSV
    if crop:
        hsv_frame = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2HSV)
    else:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Iterate through all colors defined in COLOR_RANGES
    for color_name, (lower_color, upper_color) in COLOR_RANGES.items():
        # Convert the color range to NumPy arrays
        lower_color = np.array(lower_color)
        upper_color = np.array(upper_color)

        # Create a mask for the current color range
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # Find contours (ball detection)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Check if the contour area is above the threshold
            if area > area_threshold:
                # Calculate the center of the ball using moments
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    # Calculate both cX and cY coordinates
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return cX, cY, color_name

    # If no ball was detected, return None
    return None, None, None