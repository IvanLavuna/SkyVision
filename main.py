from djitellopy import tello
import cv2

import CVAlgorithms
import DPT
import DroneAlgorithms
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
        img = cv2.resize(img, (480, 360))
        # img_depth = dpt_model.predict(img)
        cup_rect = CVAlgorithms.locate_cup(img)
        if cup_rect.is_present:
            cv2.rectangle(img, (cup_rect.x, cup_rect.y), (cup_rect.x + cup_rect.width, cup_rect.y + cup_rect.height),
                          color=(0, 255, 0), thickness=4)
        cv2.imshow("Image", img)

        # drone control
        move_vals = get_keyboard_input()
        if move_vals[0] != 0 and move_vals[1] != 0 and move_vals[2] != 0 and move_vals[3] != 0:
            my_tello.send_rc_control(move_vals[0], move_vals[1], move_vals[2], move_vals[3])
        DroneAlgorithms.next_move(img, my_tello)
        cv2.waitKey(1)
        print("[info][processing_loop] heightCM:", my_tello.get_height())

def init_everything():
    # tello
    my_tello.connect()
    print("[info] battery", my_tello.get_battery())
    my_tello.streamon()
    my_tello.set_video_direction(0) # front camera
    # KeyPress module
    kp.init()


if __name__ == '__main__':
    init_everything()
    processing_loop()
