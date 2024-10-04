from robomaster import config
from robomaster import robot
from robomaster import led
from robomaster import camera
import time
import cv2
import numpy as np
from utils import get_ip_starting_with, set_arm_low, set_arm_high, set_arm_to_grab, camera_control
import threading
from scan import scanning

# Initialize Connection
ip_prefix = '192.168.76.'
config.ROBOT_IP_STR = "192.168.76.3"
config.LOCAL_IP_STR = get_ip_starting_with(ip_prefix)

# Initalize Robot
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta", sn="3JKCK5C00308HQ")
# Access the chassis and robotic arm modules
chassis = ep_robot.chassis
arm = ep_robot.robotic_arm
cam = ep_robot.camera

# Main function
def main():
    try:
        # Robot Commands
        ep_robot.get_version()
        #select_roi_from_image()
        #while True:
        scanning(ep_robot=ep_robot)
        #camera_control(ep_robot=ep_robot)
    finally:
        # Close the connection when done
        ep_robot.close()

if __name__ == '__main__':
    main()