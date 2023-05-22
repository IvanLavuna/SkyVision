import time
from djitellopy import tello
import cv2 as cv
import Algorithms
import DPT
import KeyPress as kp
from Environment import Environment
from Agent import Agent
import datetime

# global variables
my_tello = tello.Tello()
env = Environment(my_tello)
agent = Agent(env)
images_path = "/home/lavuna47/Projects/SkyVision/images"

def manual_drone_control_step(drone: tello.Tello):
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 30

    if kp.get_key("LEFT"):
        lr = -speed
    elif kp.get_key("RIGHT"):
        lr = speed

    if kp.get_key("UP"):
        fb = speed
    elif kp.get_key("DOWN"):
        fb = -speed

    if kp.get_key("w"):
        ud = 2*speed
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
    # if lr != 0 or fb != 0 or ud != 0 or yv != 0:
    drone.send_rc_control(lr, fb, ud, yv)


def processing_loop():
    dt = 0.05  # sec
    while True:
        time_point_begin = time.time()
        # 1. retrieving image
        img = my_tello.get_frame_read().frame
        img = cv.resize(img, (420, 360))
        cv.imshow("Image", img)
        # 2. updating env
        env.AddImage(img)

        # 3. drone manual control
        manual_drone_control_step(my_tello)

        # 4. agent control
        # agent.next_move()

        # 5. wait 1 ms. Stabilization?
        cv.waitKey(1)

        # 6. save image if z was pressed
        if kp.get_key('z'):
            cv.imwrite(f"{images_path}/helipad/helipad_{datetime.datetime.now().strftime('%Y.%m.%d.%H.%M.%S')}.jpg", img)
            time.sleep(1)

        time_point_end = time.time()

        if time_point_end - time_point_begin < dt:
            time_to_sleep = dt - (time_point_end - time_point_begin)
            time.sleep(time_to_sleep)


def init_everything():
    # tello
    my_tello.connect()
    print("[info] battery", my_tello.get_battery())
    my_tello.streamon()
    # set video direction from FORWARD camera
    my_tello.set_video_direction(0)
    time.sleep(1)
    # KeyPress module
    kp.init()


if __name__ == '__main__':
    init_everything()
    processing_loop()
