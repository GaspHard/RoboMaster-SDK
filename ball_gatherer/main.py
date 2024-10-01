from robomaster import config
from robomaster import robot
from robomaster import led
from robomaster import camera
import time
import cv2
import numpy as np

config.LOCAL_IP_STR = "192.168.5.87"
config.ROBOT_IP_STR = "192.168.5.3"
ep_robot = robot.Robot()

ep_robot.initialize(conn_type="sta", sn="3JKCK5C00308HQ")

ep_version = ep_robot.get_version()
print("Robot Version: {0}".format(ep_version))
