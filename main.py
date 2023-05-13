from djitellopy import tello
import cv2 as cv
import CVAlgorithms
import KeyPress as kp
from Environment import Environment
from Agent import Agent
from DPT import dpt_model
import numpy as np

# global variables
my_tello = tello.Tello()
env = Environment(my_tello)
agent = Agent(env)


def manual_drone_control_step(drone: tello.Tello):
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kp.get_key("LEFT"):
        lr = -speed
    elif kp.get_key("RIGHT"):
        lr = speed

    if kp.get_key("UP"):
        fb = speed
    elif kp.get_key("DOWN"):
        fb = -speed

    if kp.get_key("w"):
        ud = speed
    elif kp.get_key("s"):
        ud = -speed

    if kp.get_key("a"):
        yv = -speed
    elif kp.get_key("d"):
        yv = speed

    if kp.get_key('q'):
        drone.land()

    if kp.get_key('e'):
        drone.takeoff()

    # give chance for user to interfere
    if lr != 0 or fb != 0 or ud != 0 or yv != 0:
        drone.send_rc_control(lr, fb, ud, yv)


def processing_loop():
    while True:
        # 1. retrieving image
        img = my_tello.get_frame_read().frame
        img = cv.resize(img, (480, 360))

        # 2. updating env
        env.AddImage(img)

        # 3. visualization
        cup_rect = CVAlgorithms.locate_cup(img)
        if cup_rect.is_present:
            cv.rectangle(img, (cup_rect.x, cup_rect.y), (cup_rect.x + cup_rect.width, cup_rect.y + cup_rect.height),
                         color=(0, 255, 0), thickness=4)
        cv.imshow("Image", img)

        # 4. drone manual control
        manual_drone_control_step(my_tello)

        # 5. agent control
        # agent.next_move(env)

        # 6. wait 1 ms. Stabilization?
        cv.waitKey(1)

        # 7. Debugging


def init_everything():
    # tello
    my_tello.connect()
    print("[info] battery", my_tello.get_battery())
    my_tello.streamon()
    my_tello.set_video_direction(0)  # front camera
    # KeyPress module
    kp.init()


if __name__ == '__main__':
    init_everything()
    processing_loop()
