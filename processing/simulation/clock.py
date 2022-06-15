import time


class Clock():
    def __init__(self):
        self.dt = 0
        self.last_non_zero_dt = 0
        self.time_previous = 0
        self.time_start = time.time()
        self.time_running = time.time()

    def update(self):
        self.time_running = time.time() - self.time_start
        self.dt = self.time_running - self.time_previous
        self.time_previous = self.time_running

    def reset(self):
        self.dt = 0
        self.last_non_zero_dt = 0
        self.time_previous = 0
        self.time_start = time.time()
        self.time_running = time.time()

    def get_dt(self):
        return self.dt

    def get_time_running(self):
        return self.time_running
