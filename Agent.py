from Environment import Environment
from Algorithms import Algorithms
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

    def next_move(self, env: Environment):
        # update environment
        self._env = env
        # execute job for current state
        self._jobs[self._cur_state]()
    def get_state(self):
        return self._cur_state

    def _initialization_job(self):
        pass

    def _find_cup_job(self):
        pass

    def _pick_up_cup_job(self):
        pass

    def _put_down_cup_job(self):
        pass

    def _land_job(self):
        pass

    def _final_job(self):
        pass