import time
import logging
from typing import Any

import Algorithms
from Environment import Environment
from DPT import dpt_model
import numpy as np
import cv2 as cv
import datetime
import math
import KeyPress as kp
import random


class Agent:
    """
    :brief: - Defines states and transitions between them.
            In each state Agent executes some job. Result job may impact environment and transition
            to new state.
    """
    # possible states of the Agent
    _initialization_state = "initialization state"
    _find_cup_state = "find cup state"
    _pick_up_cup_part1_state = "pick up cup part1 state"
    _pick_up_cup_part2_state = "pick up cup part2 state"
    _pick_up_cup_part3_state = "pick up cup part3 state"
    _search_for_helipad_state = "search for helipad state"
    _put_down_cup_state = "put down cup state"
    _wait_until_human_will_take_cup_state = "wait until human will take cup state"
    _land_state = "land state"
    _final_state = "final state"
    _manual_control_state = "manual control state"

    # Set up logger
    CONSOLE_HANDLER = logging.StreamHandler()
    LOG_FILE_NAME = f"logs/{datetime.datetime.now().strftime('%Y.%m.%d.%H.%M.%S')}.log"
    FILE_HANDLER = logging.FileHandler(LOG_FILE_NAME)
    FORMATTER = logging.Formatter('[%(asctime)s] - [%(levelname)s] - {%(message)s}')
    CONSOLE_HANDLER.setFormatter(FORMATTER)
    FILE_HANDLER.setFormatter(FORMATTER)

    LOGGER = logging.getLogger('Agent')
    LOGGER.addHandler(CONSOLE_HANDLER)
    LOGGER.addHandler(FILE_HANDLER)
    LOGGER.setLevel(logging.DEBUG)

    # some constants
    TELLO_CAMERA_DOWNWARD = 1
    TELLO_CAMERA_FORWARD = 0
    _downward_camera_center = (200, 180)

    def __init__(self, env: Environment):
        self._cur_state = self._initialization_state
        self._env = env
        self._jobs = {
            self._initialization_state: self._initialization_job,
            self._find_cup_state: self._find_cup_job,
            self._pick_up_cup_part1_state: self._pick_up_cup_part1_job,
            self._pick_up_cup_part2_state: self._pick_up_cup_part2_job,
            self._pick_up_cup_part3_state: self._pick_up_cup_part3_job,
            self._put_down_cup_state: self._put_down_cup_job,
            self._land_state: self._land_job,
            self._final_state: self._final_job,
            self._wait_until_human_will_take_cup_state: self._wait_until_human_will_take_cup_job,
            self._manual_control_state: self._manual_control_job,
            self._search_for_helipad_state: self._search_for_helipad_job,
        }
        # key is a tuple (x, y, z) that indicates drone position
        self._exploratory_map = set()
        self._cur_drone_pos = (0, 0, 0)
        self._cur_camera_direction = self.TELLO_CAMERA_FORWARD
        self._land_job_complete = False

    def next_move(self):
        # execute job for current state
        self._jobs[self._cur_state]()

    def get_state(self):
        return self._cur_state

    def _wait_until_human_will_take_cup_job(self):
        """
        :brief: waits until human will take a cup from drone. Uses bottom camera to figure it out
        :transition: _land_state
        """
        if self._cur_camera_direction != self.TELLO_CAMERA_DOWNWARD:
            self.__update_camera_direction(self.TELLO_CAMERA_DOWNWARD)
            return
        self._env.drone.send_rc_control(0, 0, 0, 0)
        pass

    def _initialization_job(self):
        """
        :brief: take off
        :outcome: drone is on the fly
        """
        self.LOGGER.debug("Initializing start")
        if not self._env.drone.is_flying:
            self._env.drone.takeoff()
            time.sleep(3)  # give some time for tello to take off
        self._cur_drone_pos = (0, 0, 0)
        self.__change_state(self._cur_state, self._find_cup_state)
        self.LOGGER.debug("Initializing end")

    def _manual_control_job(self):
        self.LOGGER.debug("[_manual_control_job]")
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
            ud = 2 * speed
        elif kp.get_key("s"):
            ud = -speed

        if kp.get_key("a"):
            yv = -speed
        elif kp.get_key("d"):
            yv = speed

        if kp.get_key('q'):
            self._env.drone.land()

        if kp.get_key('e'):
            self._env.drone.takeoff()

        # give chance for user to interfere
        # if lr != 0 or fb != 0 or ud != 0 or yv != 0:
        self._env.drone.send_rc_control(lr, fb, ud, yv)

    # hyperparameters & variables of job state
    _min_cup_rect_area = 300
    _should_move_dist = 60
    _drone_job_height = 40
    _success_timer = time.time()
    _first_time = False
    _last_fc_action = 0
    _current_fc_action = 0
    _action_timer = time.time()

    def _find_cup_job(self):
        """
        :brief:  Searches for cup by randomly picking directing and avoiding obstacles
        :outcome: Position of cup on the image is somehow in the middle. Area of cup rectangle >= 2000
        :Note!: There should be only 1 red object in the room in order NOT to confuse agent.
        """

        if self._env.drone.get_height() > self._drone_job_height:
            self.LOGGER.debug("cur height : {}".format(self._env.drone.get_height()))
            self._env.drone.send_rc_control(0, 0, -20, 0)
            return

        cur_img = self._env.GetLastImage()
        rect = Algorithms.locate_cup(cur_img)

        if rect.is_present:  # cup was found
            cv.rectangle(cur_img, (rect.x, rect.y), (rect.x + rect.width, rect.y + rect.height), (255, 0, 0), 3)
            cv.imshow("[_find_cup_job]", cur_img)

            self.LOGGER.debug("[_find_cup_job] CUP WAS FOUND!")
            if not self._first_time:
                self._success_timer = time.time()
                self._first_time = True
            if time.time() - self._success_timer >= 1:
                self._env.drone.send_rc_control(0, 0, 0, 0)
                self.__change_state(self._cur_state, self._pick_up_cup_part1_state)
            else:
                self._env.drone.send_rc_control(0, 0, 0, 0)
        else:
            self._first_time = False

            # choose action
            # 1. move forward
            # 2. rotate x degrees clockwise
            # 3. rotate x degrees counterclockwise
            # 4. rotate for some time cover major piece of land

            # predict depth map in order to avoid obstacles
            depth_map = dpt_model.predict(cur_img)
            # cv.imshow("Depth map cropped", depth_map)  # debug
            distance_cm = self.__get_distance_as_int(depth_map)
            self.LOGGER.debug("[find cup job] distance {}".format(distance_cm))
            if distance_cm >= self._should_move_dist:  # means no obstacles going forward
                self.LOGGER.debug("[find cup job] move forward")
                self.__move_for(0, 20, 0, 0, timeSec=0.5)
                self.__stabilize()
            else:  # there is some obstacle
                # rotate
                self.LOGGER.debug("[find cup job] rotate")
                self.__move_for(0, 0, 0, 40, timeSec=3)
                self.__stabilize()


    def _pick_up_cup_part1_job(self):
        """
        :brief: Assumes that cup was found. Pick up cup means hook it and fly up on some distance above ground
        :transition: -> _pick_up_cup_part2_state
        :algo:  1. minimize distance between drone and cup until specific point
                2. by some parabola trajectory -> try to pick up cup [only 1 try!]
                3. transition into _fly_above_state
        """
        self.LOGGER.debug("[_pick_up_cup_part1_job]")

        # debug show current image
        cv.imshow("Cur Image", self._env.GetLastImage())

        img = self._env.GetLastImage()

        cup_rect = Algorithms.locate_cup(img)
        if cup_rect.is_present:  # should be true
            cv.rectangle(img, (cup_rect.x, cup_rect.y), (cup_rect.x + cup_rect.width, cup_rect.y + cup_rect.height), (255, 0, 0), 3)
            cv.imshow("[_pick_up_cup_part1_job]", img)
            x_mid = int((cup_rect.x + cup_rect.x + cup_rect.width)/2)
            y_mid = int((cup_rect.y + cup_rect.y + cup_rect.height)/2)
            self.LOGGER.debug("[_pick_up_cup_part1_job] x: {}, y: {}".format(x_mid, y_mid))

            if x_mid < img.shape[0] * 0.45:
                self.__move_for(0, 0, 0, -20, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part1_job] rotating left")
                self.__stabilize()
            elif x_mid > img.shape[0] * 0.55:
                self.__move_for(0, 0, 0, 20, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part1_job] rotating right")
                self.__stabilize()
            if y_mid > img.shape[1] * 0.8 and cup_rect.width * cup_rect.height >= 2000:
                self.__move_for(0, 0, -20, 0, timeSec=0.1)
                self.LOGGER.debug("[_pick_up_cup_part1_job] moving down, cup area: {}".format(cup_rect.width * cup_rect.height))
                self.__stabilize()
            elif y_mid < img.shape[1] * 0.3:
                self.__move_for(0, 0, 20, 0, timeSec=0.1)
                self.LOGGER.debug("[_pick_up_cup_part1_job] moving up")
                self.__stabilize()
            else:
                self.__move_for(0, 15, 0, 0, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part1_job] moving forward")
                self.__stabilize()
        else:
            if self._env.drone.get_height() > 20:
                self.LOGGER.debug("[_pick_up_cup_part1_job] Moving down")
                self.__move_for(0, 0, -15, 0, timeSec=0.3)
                self.__stabilize()
            else:
                self.__stabilize()
                self.__move_for(0, 0, -5, 0, timeSec=5)
                self.__change_state(self._cur_state, self._pick_up_cup_part2_state)

    def _pick_up_cup_part2_job(self):
        """
        :transition: -> _pick_up_cup_part3_state
        :return:
        """
        self.LOGGER.debug("[_pick_up_cup_part2_job]")

        # debug show current image
        cv.imshow("Cur Image", self._env.GetLastImage())

        img = self._env.GetLastImage()

        cup_rect = Algorithms.locate_cup(img)
        if cup_rect.is_present:
            cv.rectangle(img, (cup_rect.x, cup_rect.y), (cup_rect.x + cup_rect.width, cup_rect.y + cup_rect.height), (255, 0, 0), 3)
            ready = True
            if cup_rect.x + cup_rect.width/2 < img.shape[0] * 0.4:
                self.__move_for(0, 0, 0, -12, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] rotating left")
                self.__stabilize()
                ready = False
            if cup_rect.x + cup_rect.width/2 > img.shape[0] * 0.6:
                self.__move_for(0, 0, 0, 12, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] rotating right")
                self.__stabilize()
                ready = False
            if cup_rect.width < 150:
                self.__move_for(0, 12, 0, 0, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] moving forward")
                self.__stabilize()
                ready = False
            if cup_rect.width > 245:
                self.__move_for(0, -12, 0, 0, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] moving backward")
                self.__stabilize()
                ready = False
            if cup_rect.height < 50:
                self.__move_for(0, 0, -15, 0, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] moving down")
                self.__stabilize()
                ready = False
            if cup_rect.height > 100:
                self.__move_for(0, 0, 15, 0, timeSec=0.2)
                self.LOGGER.debug("[_pick_up_cup_part2_job] moving up")
                self.__stabilize()
                ready = False
            if ready:
                self.__stabilize()
                self.LOGGER.debug("[_pick_up_cup_part2_job] Prepare to hook up a cup")
                self.__move_for(0, 0, 0, 0, timeSec=2)
                self.__move_for(0, 40, 0, 0, timeSec=1.2)
                self.__change_state(self._cur_state, self._pick_up_cup_part3_state)

            self.LOGGER.debug("Area: {}".format(cup_rect.width * cup_rect.height))
            self.LOGGER.debug("Width: {}".format(cup_rect.width))
            self.LOGGER.debug("Height: {}".format(cup_rect.height))

        else:
            self.__move_for(0, 0, -12, 0, timeSec=0.2)
            self.LOGGER.debug("[_pick_up_cup_part2_job] moving down...")

        if kp.get_key('t'):
            self.__stabilize()
            self.LOGGER.debug("[_pick_up_cup_part2_job] Prepare to hook up a cup")
            self.__move_for(0, 0, 0, 0, timeSec=2)
            self.__move_for(0, 40, 0, 0, timeSec=1.2)
            self.__change_state(self._cur_state, self._pick_up_cup_part3_state)

        cv.imshow("[_pick_up_cup_part2_job]", img)

    def _pick_up_cup_part3_job(self):
        """
        :brief:      -> flies on a specific position and then holds
        :transition: -> wait_until_human_will_take_cup
        """
        if self._env.drone.get_height() < 100:
            self._env.drone.send_rc_control(0, 0, 90, 0)
        else:
            self._cur_state = self._wait_until_human_will_take_cup_state

    def _put_down_cup_job(self):
        pass

    def _search_for_helipad_job(self):
        """
        :brief: searches for helipad by localization through visual markers
        :transition: _land_state
        :return:
        """
        pass

    def _land_job(self):
        """
        :definition: helipad - surface on which drone is supposed to land
        :assumption: Helipad is on the radius of 0.5 meters of the center of drone on 2D.
        :brief:      Lands on the helipad
        :algo:       1. Locate helipad
                     2. Move to that position until some tuned height
                     3. Call embed tello function for landing
        :time:       ?
        """

        if self._cur_camera_direction != self.TELLO_CAMERA_DOWNWARD:
            self.__update_camera_direction(self.TELLO_CAMERA_DOWNWARD)
            return

        img = self._env.GetLastImage()
        circles = Algorithms.locate_circles(img)
        cv.circle(img, self._downward_camera_center, 3, (255, 0, 0), 5)

        if self._env.drone.get_height() >= 60:
            self._env.drone.send_rc_control(0, 0, -20, 0)
        elif len(circles) != 0:
            # debug, visualization
            for circle in circles:
                cv.circle(img, (circle.x, circle.y), circle.radius, (0, 255, 0), 2)

            # calculate rotation
            x_avg, y_avg = Algorithms.average(circles)
            yaw = self.__land_job_calculate_yaw(x_avg, y_avg)
            self.LOGGER.debug("yaw: {}".format(yaw))
            dist_to_center = math.dist(self._downward_camera_center, (x_avg, y_avg))
            self.LOGGER.debug("dist_to_center: {}".format(dist_to_center))
            self.LOGGER.debug("height: {}".format(self._env.drone.get_height()))
            if dist_to_center <= 20:
                self._env.drone.land()
                self._land_job_complete = True
            elif dist_to_center <= 100:
                self.LOGGER.debug("dist_to_center <= 100")
                fb = -int(5 * math.cos(math.radians(yaw)))
                lr = -int(5 * math.sin(math.radians(yaw)))
                self.LOGGER.debug("lr = {}, fb = {}".format(lr, fb))
                self._env.drone.send_rc_control(lr, fb, 0, 0)
            elif abs(yaw) <= 20:  # can move backward
                self._env.drone.send_rc_control(0, -10, 0, 0)  # move backward
            else:
                if yaw < 0:
                    self._env.drone.send_rc_control(0, 0, 0, -10)  # rotate
                else:
                    self._env.drone.send_rc_control(0, 0, 0, 10)  # rotate
        else:
            self._env.drone.send_rc_control(0, 0, 0, 0)

        cv.imshow("Downward camera img", img)

    def _final_job(self):
        pass

    """
        UTILS
    """

    def __stabilize(self):
        self.__move_for(0, 0, 0, 0, timeSec=0.2)

    def __update_camera_direction(self, new_direction: int):
        if self._cur_camera_direction == new_direction:
            return
        self._cur_camera_direction = new_direction
        self._env.drone.set_video_direction(new_direction)

    def __change_state(self, prev_state, new_state):
        self._cur_state = new_state
        self.LOGGER.info('State changed : `{}` -> `{}`'.format(prev_state, new_state))

    def __get_distance_as_int(self, depth_map: np.ndarray) -> int:
        """
        :brief: cropping is done as part of this implementation
        :param depth_map: depth map of some image
        :return: value [0: 255] which tells distance in cm to object
        """
        depth_map = depth_map[int(depth_map.shape[0] * 0.4): int(depth_map.shape[0] * 0.6),
                              int(depth_map.shape[1] * 0.4): int(depth_map.shape[1] * 0.6)]
        depth_map = 255 - depth_map
        distance_cm = np.min(depth_map)
        return distance_cm

    def __land_job_calculate_yaw(self, x, y) -> int:
        """
        :return: [-180 : +180] value - degrees needed to rotate
        """
        c_x = x - self._downward_camera_center[0]
        c_y = y - self._downward_camera_center[1]

        radians = math.atan2(c_x, c_y) - math.pi / 2
        degrees = int(math.degrees(radians))
        degrees = 360 + degrees
        if degrees > 180:
            return 360 - degrees
        return degrees

    def __move_for(self, lr: int, fb: int, ud: int, yaw: int, timeSec: float):
        cur_time = time.time()
        while time.time() - cur_time <= timeSec:
            self._env.drone.send_rc_control(lr, fb, ud, yaw)

    def __get_last_image_where_cup_was_found(self) -> np.ndarray | None:
        image_list = self._env.GetImages()
        for _, img in reversed(image_list[max(0, len(image_list) - 100):]):
            cup_rect = Algorithms.locate_cup(img)
            if cup_rect.is_present:
                return img
        return None
