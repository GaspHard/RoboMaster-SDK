from robomaster import robot, camera, config
import socket
import cv2

COLOR_RANGES = {
    "yellow": ([25, 150, 150], [35, 255, 255]),  # Yellow HSV range
    "blue": ([95, 150, 150], [110, 255, 255]),   # Blue HSV range
    "green": ([55, 150, 150], [65, 255, 255]),   # Green HSV range
    "red": ([0, 150, 150], [10, 255, 255])       # Red HSV range
}

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

def camera_control(cam, arm):
    print("Starting camera preview...")

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

    # Stop the video stream and close OpenCV window
    cam.stop_video_stream()
    cv2.destroyAllWindows()