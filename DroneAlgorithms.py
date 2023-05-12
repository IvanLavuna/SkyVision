import time

import numpy as np
from djitellopy import tello

# 1. A*, whatever
# 2. Some coordination while flying (I don't believe it flies perfectly)

objectives = ["find cup", "pick up cup", "put down cup", ""]
done = False
def next_move(img: np.ndarray, drone: tello.Tello):
    global done
    """
    Assumptions:
        - assuming drone position is a point (x, y, z)
        - assuming drone flies perfectly by his control
    Algorithm:
        - Depends on the objective
    """
    if done:
        return
    drone.takeoff()
    drone.move_forward(50)
    time.sleep(3)
    drone.move_left(50)
    time.sleep(3)
    drone.move_back(50)
    time.sleep(3)
    drone.move_right(50)
    time.sleep(3)
    drone.land()
    done = True
