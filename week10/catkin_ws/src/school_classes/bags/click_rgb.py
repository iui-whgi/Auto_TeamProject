#!/usr/bin/env python3
import cv2

# Load the image saved from ROS
img = cv2.imread('saved_image.jpg')

# Mouse callback function
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = img[y, x]
        print(f"Clicked at ({x}, {y}) - R: {r}, G: {g}, B: {b}")

# Show image and set callback
cv2.imshow('Image', img)
cv2.setMouseCallback('Image', click_event)

cv2.waitKey(0)
cv2.destroyAllWindows()
