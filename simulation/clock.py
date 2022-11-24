import time
from typing import Optional


class Clock:
    def __init__(self, in_real_time: bool = True, sim_dt: Optional[float] = None):
        self._dt = 0
        self._in_real_time = in_real_time

        self._real_dt = 0
        self._real_time_previous = 0
        self._time_start = time.time()
        self._real_time_running = 0  # actual time from the start of the simulation in seconds
        if sim_dt is not None and sim_dt > 0:
            self._sim_dt = sim_dt

        self._time_running = 0  # time from the start of the simulation with change of speed accounted for
        self._speed = 1

    def update(self):
        if self._in_real_time:
            self._real_time_running = time.time() - self._time_start
            self._real_dt = self._real_time_running - self._real_time_previous
            self._dt = self._real_dt * self._speed
        else:
            self._dt = self._sim_dt * self._speed
        self._time_running += self._dt
        self._real_time_previous = self._real_time_running

    def reset(self):
        self._dt = 0
        self._real_dt = 0
        self._real_time_previous = 0
        self._time_start = time.time()
        self._real_time_running = 0
        self._time_running = 0
        self._speed = 1

    def set_speed(self, new_speed):
        self._speed = new_speed

    def set_sim_dt(self, new_sim_dt):
        self._sim_dt = new_sim_dt

    def activate_simulation_time(self, sim_dt: Optional[float] = None):
        self._in_real_time = False
        if sim_dt == 0:
            raise ValueError("sim_dt has to be greater than 0.")
        if sim_dt is not None:
            self._sim_dt = sim_dt
        elif self._sim_dt is None or self._sim_dt == 0:
            raise ValueError("sim_dt has to be preset if passed as 'None' in Clock.activate_simulation_time().")

    def deactivate_simulation_time(self):
        self._in_real_time = True

    @property
    def dt(self):
        return self._dt

    @property
    def time_running(self):
        return self._time_running

    @property
    def real_time_running(self):
        return self._real_time_running

    @property
    def sim_dt(self):
        return self._sim_dt

    @property
    def speed(self):
        return self._speed

    @property
    def in_real_time(self):
        return self._in_real_time
