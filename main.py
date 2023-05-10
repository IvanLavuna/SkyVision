from djitellopy import tello
import cv2

import CVAlgorithms
import DPT
import KeyPress as kp
import time

my_tello = tello.Tello()
dpt_model = DPT.DPTModel()


def get_keyboard_input() -> [int, int, int, int]:
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
        my_tello.land()

    if kp.get_key('e'):
        my_tello.takeoff()

    return [lr, fb, ud, yv]


def processing_loop():
    while True:
        img = my_tello.get_frame_read().frame
        img = cv2.resize(img, (540, 480))
        img_depth = dpt_model.predict(img)
        cv2.imshow("Image", img_depth)
        move_vals = get_keyboard_input()
        my_tello.send_rc_control(move_vals[0], move_vals[1], move_vals[2], move_vals[3])
        cv2.waitKey(1)


def init_everything():
    # tello
    my_tello.connect()
    print("[info] battery", my_tello.get_battery())
    my_tello.streamon()
    # KeyPress module
    kp.init()


if __name__ == '__main__':
    init_everything()
    processing_loop()
