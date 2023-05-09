from djitellopy import tello
import cv2

my_tello = tello.Tello()

def processing_loop():
    while True:
        img = my_tello.get_frame_read().frame
        cv2.imshow("Image", img)
        cv2.waitKey(1)


def init_tello():
    my_tello.connect()
    print("[info] battery", my_tello.get_battery())
    my_tello.streamon()


if __name__ == '__main__':
    init_tello()
    processing_loop()
