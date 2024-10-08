from robomaster import robot, camera, config
import socket
import cv2
import time
import numpy as np
import math

DEBUG=False
COLOR_RANGES = {
    "blue": ([95, 150, 150], [110, 255, 255]),   # Blue HSV range
    "green": ([55, 150, 100], [65, 255, 255]),   # Green HSV range
    "red": ([0, 150, 115], [12, 255, 255]),      # Red HSV range
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

def set_arm_calib(arm):
    arm.moveto(x=74, y=57).wait_for_completed()

def set_arm_low(arm):
    arm.moveto(x=181, y=-19).wait_for_completed()

def set_arm_high(arm):
    arm.moveto(x=110, y=100).wait_for_completed()

def set_arm_to_grab(arm):
    #arm.moveto(x=185, y=-84).wait_for_completed()
    arm.moveto(x=185, y=-64).wait_for_completed()

def set_arm_to_store(arm):
    arm.moveto(x=98, y=148).wait_for_completed()

def camera_control(ep_robot):
    print("Starting camera preview...")
    cam = ep_robot.camera
    arm = ep_robot.robotic_arm
    gripper = ep_robot.gripper

    # Start streaming from the camera
    cam.start_video_stream(display=False, resolution=camera.STREAM_720P)#STREAM_540P)

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

        if key == ord('s'):
            print("store height...")
            set_arm_to_store(arm)  # Call the function to lower the arm
        
        if key == ord('u'):
            print("grab height...")
            set_arm_to_grab(arm)  # Call the function to lower the arm

        if key == ord('d'):
            print("detect...")
            detect_ball(ep_camera=cam, crop=True,x=400, w=480, y=0,h=720)
            #detect_ball(ep_camera=cam)  # Call the function to lower the arm

        if key == ord('g'):
            print("gripping...")
            # Step 3: Move the arm down to grab the ball
            print("Moving arm to grab position...")
            set_arm_to_grab(arm)  # Adjust these coordinates as needed
            time.sleep(1)

            grab_ball(arm=arm, gripper=gripper, cam=cam)  # Call the function to lower the arm
            # Step 5: Check the camera to see if the ball is still visible
            

    # Stop the video stream and close OpenCV window
    cam.stop_video_stream()
    cv2.destroyAllWindows()


def grab_ball(arm, gripper, cam):
    """
    Grabs a ball using the gripper, and checks if the ball is successfully grabbed using resistance feedback or the camera.
    
    Args:
    arm (robomaster.robotic_arm): The robotic arm object to control the gripper arm.
    gripper (robomaster.gripper): The gripper object to grab the ball.
    cam (robomaster.camera): The camera object to verify if the ball is grabbed.
    
    Returns:
    bool: True if the ball is successfully grabbed, False otherwise.
    """
    print("Opening the gripper...")
    gripper.open(power=50)  # Adjust power as necessary
    time.sleep(1.5)
    print("Closing the gripper to grab the ball...")
    gripper.close(power=50)  # Close the gripper with 50% power

    for _ in range(1):  # Check a few frames to confirm
        # first frame is always wrong
        for i in range(3):
            frame = cam.read_cv2_image(strategy='newest')
        time.sleep(2)
        frame = cam.read_cv2_image(strategy='newest')
        frame = cv2.resize(frame, (960, 540))

        if frame is not None:
            # Detect ball in grapper area 
            color = detect_ball(frame=frame, crop = True, x=390, y=300, w=200, h=190, area_threshold_red=7000, area_threshold_rest=7000)[2]
            if color is not None:
                print(f"{color} ball grabbed successfully")
                return color
            else:
                return None
    

def open_gripper(gripper):
    print("Opening the gripper...")
    gripper.open(power=50)  # Adjust power as necessary
    time.sleep(2)


# Function to calculate the Euclidean distance between two points
def calculate_distance(c1, c2):
    return math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

# Function to calculate the distance from the center of the frame
def distance_from_center(cX, cY, frame_width, frame_height):
    center_x, center_y = frame_width // 2, frame_height // 2
    return math.sqrt((cX - center_x) ** 2 + (cY - center_y) ** 2)

# Function to detect a red ball in the camera frame
# Function to detect the largest ball of any color defined in COLOR_RANGES
# Function to detect a red ball in the camera frame
def detect_ball(ep_camera=None, frame=None, area_threshold_rest=50, vertices_red = 5, vertices_rest = 4, area_threshold_red=300, max_area = 35000, circularity_threshold_red=0.4,  circularity_threshold_rest=0.15, crop=False, x=0, y=0, h=0, w=0):
    if frame is None:
        # Read the latest frame from the camera
        frame = ep_camera.read_cv2_image(strategy='newest')
    
    height, width, channels = frame.shape
    if height == 720 and width == 1280:
        frame = apply_mask_to_image(frame=frame, mask_path="ball_gatherer\\mask_low.png")
        #print("applying mask low")
        #cv2.imshow("masked frame", frame)
        #cv2.waitKey(0)
    #print(f"Height {height}, Width {width}, Channels {channels}")

    # Convert the frame or cropped frame to HSV
    if crop:
        # Apply cropping
        roi_frame = frame[y:y+h, x:x+w]
        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        height, width, channels = hsv_frame.shape
        #print(f"Height {height}, Width {width}, Channels {channels}")
    else:
        # No cropping
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if DEBUG and crop:
        cv2.imshow("HSV Frame", hsv_frame)  # Show the frame or cropped frame for debugging

    leftmost_ball_per_color = []

    # Iterate through all colors defined in COLOR_RANGES
    for color_name, (lower_color, upper_color) in COLOR_RANGES.items():
        if color_name == "red":
            area_threshold = area_threshold_red
            circularity_threshold = circularity_threshold_red
            vertices_threshold = vertices_red
        else:
            area_threshold = area_threshold_rest
            circularity_threshold = circularity_threshold_rest
            vertices_threshold = vertices_rest
        # Convert the color range to NumPy arrays
        lower_color = np.array(lower_color)
        upper_color = np.array(upper_color)

        # Create a mask for the current color range
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)
        
        # Perform morphological operations to remove small noise in the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours (ball detection)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            sorted_contours = sorted(contours, key=lambda contour: cv2.boundingRect(contour)[0])
            for c in sorted_contours:
                area = cv2.contourArea(c)
                perimeter = cv2.arcLength(c, True)

                if perimeter > 0:
                    # Calculate circularity to determine roundness
                    circularity = 4 * np.pi * (area / (perimeter ** 2))

                    #if DEBUG:
                    #print(f"Color: {color_name}, Area: {area}, Circularity: {circularity}")

                    # Only proceed if area and circularity thresholds are met
                    if area > area_threshold and circularity > circularity_threshold:
                        leftmost_contour = c
                        break

            # Check if the contour meets the thresholds
            if area > area_threshold and area < max_area and circularity > circularity_threshold:
                approx = cv2.approxPolyDP(c, 0.02 * perimeter, True)
                #print(f"approximation vertices: {len(approx)}")
                if len(approx) >= vertices_threshold:  # More vertices indicate round shape
                    # Calculate the center and radius of the ball using min enclosing circle
                    (cX, cY), radius = cv2.minEnclosingCircle(leftmost_contour)

                    # Convert to integers
                    cX = int(cX)
                    cY = int(cY)
                    radius = int(radius)

                    if crop:
                        cX += x
                        cY += y

                    # Draw the detected ball on the original frame for debugging
                    cv2.circle(frame, (cX, cY), radius, (0, 255, 0), 2)

                    leftmost_ball_per_color.append([cX, cY, color_name, area])

    if len(leftmost_ball_per_color) > 0:
        if len(leftmost_ball_per_color) >= 2:
            leftmost_ball_per_color = [ball for ball in leftmost_ball_per_color if ball[2] != "red"]

        if DEBUG:
            cv2.imshow("Detected Ball", frame)
            cv2.waitKey(0)

        # Find the leftmost ball
        leftmost_ball = min(leftmost_ball_per_color, key=lambda x: x[0])
        #if DEBUG:
        #print(leftmost_ball)
        return leftmost_ball[0:3]
    else:
        return None, None, None


def apply_mask_to_image(image_path = None, mask_path = None, frame = None):
    """
    Apply a mask to the given image.
    
    Args:
        image_path: Path to the image file.
        mask_path: Path to the mask image file (black and white).
    
    Returns:
        masked_image: The image with the mask applied.
    """
    # Load the image
    if frame is None:
        image = cv2.imread(image_path)
    else:
        image = frame

    # Load the mask image (ensure it is grayscale)
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

    # Ensure the image and mask have the same dimensions
    if image.shape[:2] != mask.shape[:2]:
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))

    # Apply the mask using cv2.bitwise_and()
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    
    return masked_image