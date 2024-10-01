from robomaster import config
from robomaster import robot
import socket

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