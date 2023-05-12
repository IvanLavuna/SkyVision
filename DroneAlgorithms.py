import time
import numpy as np
from djitellopy import tello
import CVAlgorithms
from queue import Queue
from Environment import Environment

# 1. A*, whatever
# 2. Some coordination while flying (I don't believe it flies perfectly)

objectives = {"initialize drone": False,
              "find cup": False,
              "pick up cup": False,
              "put down cup": False,
              "land": False}

frames = Queue()


def next_move(env: Environment):
    global objectives
    """
    Assumptions:
        - assuming drone position is a point (x, y, z)
        - assuming drone flies perfectly by his control
    Algorithm:
        - Depends on the objective
    """
    if not objectives["initialize drone"]:
        initialize_drone(env)
    elif not objectives["find cup"]:
        find_cup(env)
    elif not objectives["pick up cup"]:
        pick_up_cup(env)
        # print("[info] objective: 'pick up cup' done!")
    elif not objectives["put down cup"]:
        put_down_cup(env)
        # print("[info] objective: 'put down cup' done!")
    elif not objectives["land"]:
        land(env)
        # print("[info] objective: 'landed' done!")
    else:
        # print("[info] all objectives done!")
        pass


def find_cup(env: Environment):
    global objectives
    cup_rect = CVAlgorithms.locate_cup(env.GetLastImage())
    # todo: some movements with stabilization
    if not cup_rect.is_present:
        env.drone.send_rc_control(0, 0, 0, 35)
        print("[debug]", "rotating...")
        return

    objectives["find cup"] = True
    print("[info] objective: 'pick up cup' done!")


def pick_up_cup(env: Environment):
    '''
    :brief Assumes cup is in the current drone vision
    :goal Take cup on the hook and take off with it on 100 cm
    '''
    cup_rect = CVAlgorithms.locate_cup(env.GetLastImage())
    if not cup_rect.is_present:
        print("[debug]", "rotating...")
        return
    env.drone.send_rc_control(0, 20, 0, 0)
    my_list = list()

    print("[info][pick_up_cup]: moving forward...")
    pass


def put_down_cup(env: Environment):
    pass


def land(env: Environment):
    pass


def stabilize(drone: tello.Tello, timeSec: int):
    cur_tp = time.time()
    while time.time() - cur_tp < timeSec:
        drone.send_rc_control(0, 0, 0, 0)
        print("[debug]", "stabilizing...")
        time.sleep(0.1)


def initialize_drone(env: Environment):
    env.drone.takeoff()
    time.sleep(3)
    set_drone_height(env.drone, 40)
    objectives["initialize drone"] = True
    stabilize(env.drone, 4)
    print("[info] objective: 'initialize drone' done!")


def set_drone_height(drone: tello.Tello, cm_height: float):
    while drone.get_height() < cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, 20, 0)
    drone.send_rc_control(0, 0, 0, 0)
    while drone.get_height() > cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, -20, 0)
    drone.send_rc_control(0, 0, 0, 0)
