import time

from djitellopy import tello
import cv2
import numpy as np
import logging


class Environment:
    '''
    :brief source of data for algorithm
        Contains following data:
        :image_list - list of tuples (timestamp, image)
        :drone - tello object
    :note! This class may extend if needed
    '''
    MAX_IMAGES = 100

    def __init__(self, drone: tello.Tello):
        self.drone = drone
        self.image_list = []

    def AddImage(self, img: np.ndarray):
        if len(self.image_list) == self.MAX_IMAGES:
            self.PopImage()
        self.image_list.append((time.time(), img))

    def PopImage(self):
        self.image_list.pop(0)

    def GetImageSize(self):
        return len(self.image_list)

    def GetImages(self):
        return self.image_list

    def Drone(self):
        return self.drone

    def GetLastImage(self):
        if len(self.image_list) == 0:
            raise RuntimeError("There is no images in the list!")
        return self.image_list[len(self.image_list) - 1][1]

    def GetLastImageWithTimestamp(self):
        if len(self.image_list) == 0:
            raise RuntimeError("There is no images in the list!")
        return self.image_list[len(self.image_list) - 1]
