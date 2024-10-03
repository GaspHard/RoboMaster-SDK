import cv2
import numpy as np

# Load the image you provided
image = cv2.imread("ball_gatherer\\red_ball_far.png")

# Convert the image to the HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Function to get HSV values by clicking on the image
def pick_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = hsv_image[y, x]
        print(f"HSV value at ({x}, {y}) is: {hsv_value}")

# Create a window and set a mouse callback to select HSV values
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", pick_color)

while True:
    cv2.imshow("Image", image)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up the window
cv2.destroyAllWindows()

""" Green (653, 475) is: [ 60 255 205]
Blue (867, 477) is: [102 255 255]
Red (1088, 488) is: [  0 255 251]
Yellow (1302, 492) is: [ 30 255 255] """