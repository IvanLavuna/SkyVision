from typing import Type

import cv2
import numpy as np


class CupPosition:
    """
    @brief: specifies position of a cup on an image.
    @param: (x,y) -> up left position of rectangle, where cup is located
    @param: (width, height)
    """

    def __init__(self, x=None, y=None, width=None, height=None, cup_is_present=False):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.cup_is_present = cup_is_present


def locate_cup(img: np.ndarray) -> CupPosition:
    """
    @brief: algorithm that locates cup on an image
    @param img - has shape (n, m, 3)
    """
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define a range of red color in HSV space
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    red_mask2 = cv2.inRange(hsv, lower_red, upper_red)
    red_mask = red_mask1 + red_mask2

    # Apply morphological operations to remove noise and refine the shape of the red regions
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    # Detect contours in the resulting binary image
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter the contours based on their shape to identify the red rectangle
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            if 1.5 <= w / h <= 2.5:
                return CupPosition(x=x, y=y, width=w, height=h, cup_is_present=True)
    return CupPosition()


def _locate_cup_test():
    # load few images: verify it works as expected
    pass


if __name__ == '__main__':
    _locate_cup_test()