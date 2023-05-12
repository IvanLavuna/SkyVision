from typing import Type

import cv2
import numpy as np


class Rectangle:
    """
    @brief: specifies position of a cup on an image.
    @param: (x,y) -> up left position of rectangle, where cup is located
    @param: (width, height)
    """

    def __init__(self, x=None, y=None, width=None, height=None, is_present=False):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.is_present = is_present

    def __str__(self):
        return f"MyClass(x={self.x}, y={self.y}, w={self.width}, h={self.height})"

def find_bounding_box(img: np.ndarray):
    red_regions = img[:, :, 0]  # Assuming red_regions is a separate array with shape (img.shape[0], img.shape[1])
    non_zero_indices = np.nonzero(red_regions)
    non_zero_rows, non_zero_cols = non_zero_indices[0], non_zero_indices[1]

    if len(non_zero_rows) > 0:
        l = np.min(non_zero_cols)
        r = np.max(non_zero_cols)
        u = np.min(non_zero_rows)
        d = np.max(non_zero_rows)
    else:
        # Handle case when no red regions are found
        l, r, u, d = 0, 0, 0, 0

    return l, r, u, d

def locate_cup(img: np.ndarray) -> Rectangle:
    """
    @brief: algorithm that locates cup on an image
    @param img - has shape (n, m, 3)
    """
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for red color in HSV
    lower_red = np.array([0, 150, 150])
    upper_red = np.array([10, 255, 255])

    # Create a mask to isolate red color regions
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Apply the mask to the original image
    red_regions = cv2.bitwise_and(img, img, mask=mask)
    # cv2.imshow("localte_cup debug", red_regions)

    l, r, u, d = find_bounding_box(red_regions)
    if (r - l) * (d - u) > 1000 and l != 0 and r != 0 and u != 0 and d != 0 :
        return Rectangle(l, u, r - l, d - u, True)

    return Rectangle()




if __name__ == '__main__':
    pass