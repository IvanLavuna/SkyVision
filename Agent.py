import time
import logging
import CVAlgorithms
from Environment import Environment
from Algorithms import Algorithms
from DPT import dpt_model
import numpy as np
import cv2 as cv

class Agent:
    """
    :brief: Defines states and transitions between them.
            In each state Agent executes some job. Result job may impact environment and transition
            to new state.
    """
    # possible states of the Agent
    _initialization_state = "initialization state"
    _find_cup_state = "find cup state"
    _pick_up_cup_state = "pick up cup state"
    _put_down_cup_state = "put down cup state"
    _land_state = "land state"
    _final_state = "final state"

    # Set up logger
    CONSOLE_HANDLER = logging.StreamHandler()
    FILE_HANDLER = logging.FileHandler('application.log')
    FORMATTER = logging.Formatter('[%(asctime)s] - [%(levelname)s] - {%(message)s}')
    CONSOLE_HANDLER.setFormatter(FORMATTER)
    FILE_HANDLER.setFormatter(FORMATTER)

    LOGGER = logging.getLogger('Agent')
    LOGGER.addHandler(CONSOLE_HANDLER)
    LOGGER.addHandler(FILE_HANDLER)
    LOGGER.setLevel(logging.DEBUG)

    # used in search algorithm, which divides the space into cubes
    _XY_cube_len = 40  # cm
    _Z_cube_len = 40  # cm

    # used in 'find cup' job
    _min_cup_rect_area = 2000
    _move_from_depth_map_constant = 60
    _standard_moving_delay_sec = 3

    def __init__(self, env: Environment):
        self._cur_state = self._initialization_state
        self._env = env
        self._jobs = {
            self._initialization_state: self._initialization_job,
            self._find_cup_state: self._find_cup_job,
            self._pick_up_cup_state: self._pick_up_cup_job,
            self._put_down_cup_state: self._put_down_cup_job,
            self._land_state: self._land_job,
            self._final_state: self._final_job()
        }
        # key is a tuple (x, y, z) that indicates drone position
        self._exploratory_map = set()
        self._cur_drone_pos = (0, 0, 0)

    def next_move(self, env: Environment):
        # update environment
        self._env = env
        # execute job for current state
        self._jobs[self._cur_state]()

    def get_state(self):
        return self._cur_state

    def _initialization_job(self):
        """
        :brief: Take tello into initial state - take off
        :outcome: drone is on the fly
        """
        self.LOGGER.info("Initializing...")
        if not self._env.drone.is_flying:
            self._env.drone.takeoff()
            time.sleep(4)  # give some time for tello to take off
        self._cur_state = self._find_cup_state
        self._cur_drone_pos = (0, 0, 0)
        self.LOGGER.info("Initializing complete!")
        self.LOGGER.info('State changed - cur_state: {}'.format(self._cur_state))

    def _find_cup_job(self):
        """
        :brief:   Divides searching space into blocks. 40 x 40 x 40. There will be 3 blocks(120 cm) for height though to
                  capture reality of room at home. Drone moves randomly on the bottom and with time deadline for
                  each block on the Z axis
        :outcome: Position of cup on the image is somehow in the middle. Area of cup rectangle >= 2000
        :Note!: There should be only 1 red object in the room in order NOT to confuse agent.
        """
        cur_img = self._env.GetLastImage()
        rect = CVAlgorithms.locate_cup(cur_img)
        if rect.is_present:
            # move to cup in order to make rect area >= _min_cup_rect_area
            self.LOGGER.debug("[find cup job] cup was found! Stabilizing...")
            self._env.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(1)
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
                self._env.drone.move_forward(self._XY_cube_len)
                time.sleep(self._standard_moving_delay_sec)
                # self._cur_drone_pos = (self._cur_drone_pos[0], self._cur_drone_pos[1]+1, self._cur_drone_pos[2])
                # self._exploratory_map.add(self._cur_drone_pos)
            else:
                # there are some obstacle
                self.LOGGER.debug("[find cup job] avoiding obstacle")
                self._env.drone.rotate_clockwise(90)
                time.sleep(self._standard_moving_delay_sec)

    def _pick_up_cup_job(self):
        pass

    def _put_down_cup_job(self):
        pass

    def _land_job(self):
        pass

    def _final_job(self):
        pass