import time
import numpy as np
from djitellopy import tello
import CVAlgorithms
from queue import Queue

# 1. A*, whatever
# 2. Some coordination while flying (I don't believe it flies perfectly)

objectives = {"initialize drone": False,
              "find cup": False,
              "pick up cup": False,
              "put down cup": False,
              "land": False}

frames = Queue()


def next_move(img: np.ndarray, drone: tello.Tello):
    global objectives
    """
    Assumptions:
        - assuming drone position is a point (x, y, z)
        - assuming drone flies perfectly by his control
    Algorithm:
        - Depends on the objective
    """
    if not objectives["initialize drone"]:
        initialize_drone(drone)
    elif not objectives["find cup"]:
        find_cup(img, drone)
    elif not objectives["pick up cup"]:
        pick_up_cup(img, drone)
        # print("[info] objective: 'pick up cup' done!")
    elif not objectives["put down cup"]:
        put_down_cup(img, drone)
        # print("[info] objective: 'put down cup' done!")
    elif not objectives["land"]:
        land(img, drone)
        # print("[info] objective: 'landed' done!")
    else:
        # print("[info] all objectives done!")
        pass

def find_cup(img: np.ndarray, drone: tello.Tello):
    global objectives
    cup_rect = CVAlgorithms.locate_cup(img)
    # todo: some movements with stabilization
    if not cup_rect.is_present:
        drone.send_rc_control(0, 0, 0, 35)
        print("[debug]", "rotating...")
        return

    objectives["find cup"] = True
    print("[info] objective: 'pick up cup' done!")


def pick_up_cup(img: np.ndarray, drone: tello.Tello):
    '''
    :brief Assumes cup is in the current drone vision
    :goal Take cup on the hook and take off with it on 100 cm
    '''
    cup_rect = CVAlgorithms.locate_cup(img)
    if not cup_rect.is_present:
        print("[debug]", "rotating...")
        return
    drone.send_rc_control(0, 20, 0, 0)
    my_list = list()

    print("[info][pick_up_cup]: moving forward...")
    pass

def put_down_cup(img: np.ndarray, drone: tello.Tello):
    pass

def land(img: np.ndarray, drone: tello.Tello):
    pass

def stabilize(drone: tello.Tello, timeSec: int):
    cur_tp = time.time()
    while time.time() - cur_tp < timeSec:
        drone.send_rc_control(0, 0, 0, 0)
        print("[debug]", "stabilizing...")
        time.sleep(0.1)

def initialize_drone(drone: tello.Tello):
    drone.takeoff()
    time.sleep(3)
    set_drone_height(drone, 40)
    objectives["initialize drone"] = True
    stabilize(drone, 4)
    print("[info] objective: 'initialize drone' done!")


def set_drone_height(drone: tello.Tello, cm_height: float):
    while drone.get_height() < cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, 20, 0);
    drone.send_rc_control(0, 0, 0, 0);
    while drone.get_height() > cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, -20, 0);
    drone.send_rc_control(0, 0, 0, 0);
