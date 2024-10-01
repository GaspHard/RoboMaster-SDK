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
