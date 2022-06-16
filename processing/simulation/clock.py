import time


class Clock():
    def __init__(self):
        self.dt = 0
        self.real_dt = 0
        self.real_time_previous = 0
        self.time_start = time.time()
        self.real_time_running = 0  # actual time from the start of the simulation in seconds
        self.time_running = 0  # time from the start of the simulation with change of speed accounted for
        self.speed = 1

    def update(self):
        self.real_time_running = time.time() - self.time_start
        self.real_dt = self.real_time_running - self.real_time_previous
        self.dt = self.real_dt * self.speed
        self.time_running += self.dt
        self.real_time_previous = self.real_time_running

    def reset(self):
        self.dt = 0
        self.real_dt = 0
        self.real_time_previous = 0
        self.time_start = time.time()
        self.real_time_running = 0
        self.time_running = 0
        self.speed = 1

    def change_speed(self, new_speed):
        self.speed = new_speed

    def get_dt(self):
        return self.dt

    def get_time_running(self):
        return self.time_running

    def get_real_time_running(self):
        return self.real_time_running

    def get_speed(self):
        return self.speed
