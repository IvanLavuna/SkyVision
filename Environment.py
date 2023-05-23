import time

from djitellopy import tello
import numpy as np


class Environment:
    '''
    :brief source of data for algorithm
        Contains following data:
        :image_list - list of tuples (timestamp, image)
        :drone - tello object
    :note! This class may extend if needed
    '''
    MAX_IMAGES = 1000

    def __init__(self, drone: tello.Tello):
        self.drone = drone
        self._image_list = []

    def AddImage(self, img: np.ndarray):
        if len(self._image_list) >= self.MAX_IMAGES:
            self._image_list.pop(0)
        self._image_list.append((time.time(), img))


    def PopImage(self):
        self._image_list.pop(0)

    def GetImageSize(self):
        return len(self._image_list)

    def GetImages(self):
        return self._image_list

    def Drone(self):
        return self.drone

    def GetLastImage(self) -> np.ndarray:
        if len(self._image_list) == 0:
            raise RuntimeError("There is no images in the list!")
        return self._image_list[len(self._image_list) - 1][1]

    def GetLastImageWithTimestamp(self) -> (time.time, np.ndarray):
        if len(self._image_list) == 0:
            raise RuntimeError("There is no images in the list!")
        return self._image_list[len(self._image_list) - 1]
