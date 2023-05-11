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

    l, r, u, d = 1000, -1000, 1000, -1000
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if red_regions[i][j][0] != 0:
                l = min(l, j)
                r = max(r, j)
                u = min(u, i)
                d = max(d, i)

    # square should be at least 50
    x, y = l, u
    width = r - l
    height = d - u
    if (r-l)*(d-u) > 200 and l != 1000 and r != -1000 and u != 1000 and d != -1000 :
        return Rectangle(x, y, width, height, True)

    return Rectangle()


def _locate_cup_test():
    # load few images: verify it works as expected
    pass


if __name__ == '__main__':
    _locate_cup_test()