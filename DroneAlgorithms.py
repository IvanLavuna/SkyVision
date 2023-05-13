import time
import numpy as np
from djitellopy import tello
import CVAlgorithms
from queue import Queue
from Environment import Environment
from DPT import dpt_model

# 1. A*, whatever
# 2. Some stabilization while flying (I don't believe it flies perfectly)

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
    global objectives
    """
    :brief: Assumes cup is in the current drone vision
    :goal: Take cup on the hook and take off with it on 100 cm
    :algo: 1. Preserve cup rectangle on the x midpoint axis
             - rotate left | right otherwise
           2. Move forward & stabilize until specific distance was achieved
           3. Linear move forward with some angle (30 degrees)
    """
    img = env.GetLastImage()
    rect = CVAlgorithms.locate_cup(img)
    if not rect.is_present:
        env.drone.send_rc_control(0, 0, 0, 20)
        print("[debug][pick_up_cup]", "rect was not present. rotating right...")
        return

    # 1. preserve cup rect
    x_mid, y_mid = rect.mid_point()
    if x_mid < img.shape[0] / 2 - 20:
        env.drone.send_rc_control(0, 0, 0, -20)
        print("[debug][pick_up_cup]", "preserving cup rect, moving right")
        return
    if x_mid > img.shape[0] / 2 + 20:
        env.drone.send_rc_control(0, 0, 0, 20)
        print("[debug][pick_up_cup]", "preserving cup rect, moving left")
        return

    # 2. Move forward & stabilize until specific distance was achieved
    cup_dist = get_distance_to_cup(img, rect)
    print("[debug][pick_up_cup]", "cup_dist:", cup_dist)
    if cup_dist > 50:  # cm?
        move(env.drone, 1, 0, 40, 0, 0)
        print("[debug][pick_up_cup]", "moving forward to cup")
    else:
        move(env.drone, 1, 0, 40, 10, 0)
        print("[debug][pick_up_cup]", "cup is nearby")


    # finalizing
    # objectives["pick up cup"] = True
    # print("[info][pick_up_cup]: moving forward...")
    # pass


def get_distance_to_cup(img: np.ndarray, rect: CVAlgorithms.Rectangle) -> int:
    """
    :brief: takes ~2 second to complete - expensive operation
    :param img: contains cup, where rect is defined
    :param rect: bounding box for a cup
    :return: integer value which is a distance to a cup
    """
    depth_map = dpt_model.predict(img)
    x, y, width, height = rect.x, rect.y, rect.width, rect.height

    # Extract the region of interest (ROI) from the depth map
    roi = depth_map[y:y+height, x:x+width]

    # Flatten the ROI array to calculate the average distance
    flattened_roi = roi.flatten()

    # Calculate the average distance value within the ROI
    average_distance = np.mean(flattened_roi)

    # Round the average distance and return it as an integer
    return int(round(average_distance))


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
    print("[info] objective: 'initialize drone' done!")


def move(drone: tello.Tello, timeSec: int, lr=0, fb=0, ud=0, yv=0):
    cur_tp = time.time()
    while time.time() - cur_tp < timeSec:
        drone.send_rc_control(lr, fb, ud, yv)
        print("[debug]", "moving forward...")
        time.sleep(0.1)


def set_drone_height(drone: tello.Tello, cm_height: float):
    while drone.get_height() < cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, 20, 0)
    drone.send_rc_control(0, 0, 0, 0)
    while drone.get_height() > cm_height:
        print("[debug] drone height:", drone.get_height())
        drone.send_rc_control(0, 0, -20, 0)
    drone.send_rc_control(0, 0, 0, 0)
