from typing import Type
import cv2 as cv
import numpy as np
import os

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

    def mid_point(self) -> (int, int):
        if self.is_present:
            return self.x + self.width / 2, self.y + self.height / 2

    def __str__(self):
        return f"MyClass(x={self.x}, y={self.y}, w={self.width}, h={self.height})"


class Circle:
    def __init__(self, x=None, y=None, radius=None, is_present=False):
        self.x = x
        self.y = y
        self.radius = radius
        self.is_present = is_present

    def __str__(self):
        return f"MyClass(x={self.x}, y={self.y}, radius={self.radius})"



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
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Define the lower and upper bounds for red color in HSV
    lower_red = np.array([0, 150, 150])
    upper_red = np.array([10, 255, 255])

    # Create a mask to isolate red color regions
    mask = cv.inRange(hsv, lower_red, upper_red)

    # Apply the mask to the original image
    red_regions = cv.bitwise_and(img, img, mask=mask)

    l, r, u, d = find_bounding_box(red_regions)
    if (r - l) * (d - u) > 1000 and l != 0 and r != 0 and u != 0 and d != 0:
        return Rectangle(l, u, r - l, d - u, True)

    return Rectangle()


def locate_helipad_as_circle(img: np.ndarray) -> Circle:
    """
    :param img: searching area
    :return: Circle which can be not set if helipad wasn't detected
    """
    # Convert the image to a single-channel grayscale image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Blur the image to reduce noise
    gray = cv.medianBlur(gray, 5)

    # Apply the Hough Circle Transform
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=30)

    # Ensure at least some circles were found
    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Sort circles by radius
        circles = sorted(circles, key=lambda x: -x[2])

        return Circle(x=circles[0][0], y=circles[0][1], radius=circles[0][2], is_present=True)

    return Circle()


def _locate_helipad_as_circle_test():
    helipad_dir = "/home/lavuna47/Projects/SkyVision/images/helipad"
    for filename in os.listdir(helipad_dir):
        if filename.endswith(".jpg"):
            img = cv.imread(os.path.join(helipad_dir, filename))
            locate_helipad_as_circle(img)


if __name__ == '__main__':
    _locate_helipad_as_circle_test()
