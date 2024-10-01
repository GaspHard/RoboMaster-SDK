from robomaster import config
from robomaster import robot
from robomaster import led
from robomaster import camera
import time
import cv2
import numpy as np
from utils import get_ip_starting_with

# Initialize Connection
ip_prefix = '192.168.5.'
config.ROBOT_IP_STR = "192.168.5.3"
config.LOCAL_IP_STR = get_ip_starting_with(ip_prefix)
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta", sn="3JKCK5C00308HQ")


# Main function
def main():
    try:
        # Robot Commands
        ep_version = ep_robot.get_version()
        print("Robot Version: {0}".format(ep_version))
    finally:
        # Close the connection when done
        ep_robot.close()

if __name__ == '__main__':
    main()