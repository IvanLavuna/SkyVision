import time
import logging
import Algorithms
from Environment import Environment
from DPT import dpt_model
import numpy as np
import cv2 as cv
import datetime
import math


class Agent:
    """
    :brief: - Defines states and transitions between them.
            In each state Agent executes some job. Result job may impact environment and transition
            to new state.
    """
    # possible states of the Agent
    _initialization_state = "initialization state"
    _find_cup_state = "find cup state"
    _pick_up_cup_state = "pick up cup state"
    _put_down_cup_state = "put down cup state"
    _wait_until_human_will_take_cup_state = "wait until human will take cup state"
    _fly_above_state = "fly above state"
    _land_state = "land state"
    _final_state = "final state"

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
            self._pick_up_cup_state: self._pick_up_cup_job,
            self._put_down_cup_state: self._put_down_cup_job,
            self._land_state: self._land_job,
            self._final_state: self._final_job,
            self._wait_until_human_will_take_cup_state: self._wait_until_human_will_take_cup_job,
            self._fly_above_state: self._fly_above_job
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
        :brief: For now it will just hang in the air.
                Later it should recognise from bottom camera that human took a cup
        :return:
        """
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

    # some hyperparameters for a given job
    _min_cup_rect_area = 2000
    _move_from_depth_map_constant = 60
    _standard_moving_delay_sec = 3
    _XY_cube_len = 40  # cm
    _Z_cube_len = 40  # cm

    def _find_cup_job(self):
        """
        :brief:   Divides searching space into blocks. 40 x 40 x 40. There will be 3 blocks(120 cm) for height though to
                  capture reality of room at home. Drone moves randomly on the bottom and with time deadline for
                  each block on the Z axis
        :outcome: Position of cup on the image is somehow in the middle. Area of cup rectangle >= 2000
        :Note!: There should be only 1 red object in the room in order NOT to confuse agent.
        """
        cur_img = self._env.GetLastImage()
        rect = Algorithms.locate_cup(cur_img)
        if rect.is_present:
            # move to cup in order to make rect area >= _min_cup_rect_area
            self.LOGGER.debug("[find cup job] cup was found! Stabilizing...")
            self._env.drone.send_rc_control(0, 0, 0, 0)
            self._cur_state = self._pick_up_cup_state
            pass
        else:
            # predict depth map in order to avoid obstacles
            depth_map = dpt_model.predict(cur_img)
            depth_map = depth_map[int(cur_img.shape[0] * 0.4): int(cur_img.shape[0] * 0.6),
                                  int(cur_img.shape[1] * 0.4): int(cur_img.shape[1] * 0.6)]
            depth_map = 255 - depth_map
            cv.imshow("Depth map cropped", depth_map)  # debug
            distance_cm = np.min(depth_map)
            self.LOGGER.debug("[find cup job] distance {}".format(distance_cm))
            if distance_cm >= self._move_from_depth_map_constant:  # means no obstacles going forward
                self.LOGGER.debug("[find cup job] moving forward")
                self._env.drone.send_rc_control(0, 40, 0, 0)
                time.sleep(self._standard_moving_delay_sec)
                # self._cur_drone_pos = (self._cur_drone_pos[0], self._cur_drone_pos[1]+1, self._cur_drone_pos[2])
                # self._exploratory_map.add(self._cur_drone_pos)
            else:
                # there are some obstacle
                self.LOGGER.debug("[find cup job] avoiding obstacle")
                self._env.drone.rotate_clockwise(90)
                time.sleep(self._standard_moving_delay_sec)
                self._env.drone.send_rc_control(0, 0, 0, 0)

    def _pick_up_cup_job(self):
        """
        :brief: Assumes that cup was found. Pick up cup means hook it and fly up on some distance above ground
        :transition: -> _fly_above
        :algo:  1. minimize distance between drone and cup until specific point
                2. by some parabola trajectory -> try to pick up cup [only 1 try!]
                3. transition into _fly_above_state
        """
        pass

    def _fly_above_job(self):
        """
        :brief:      -> flies on a specific position and then holds
        :transition: -> wait_until_human_will_take_cup
        """
        if self._env.drone.get_height() < 100:
            self._env.drone.send_rc_control(0, 0, 60, 0)
        else:
            self._cur_state = self._wait_until_human_will_take_cup_state

    def _put_down_cup_job(self):
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
        if self._land_job_complete:
            return
        if self._cur_camera_direction != self.TELLO_CAMERA_DOWNWARD:
            self.__update_camera_direction(self.TELLO_CAMERA_DOWNWARD)
            return

        # take off
        if not self._env.drone.is_flying:
            self._env.drone.takeoff()
            time.sleep(3)  # give some time for tello to take off

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

    def __update_camera_direction(self, new_direction: int):
        if self._cur_camera_direction == new_direction:
            return
        self._cur_camera_direction = new_direction
        self._env.drone.set_video_direction(new_direction)

    def __change_state(self, prev_state, new_state):
        self._cur_state = new_state
        self.LOGGER.info('State changed : `{}` -> `{}`'.format(prev_state, new_state))

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
