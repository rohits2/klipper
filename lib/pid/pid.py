from collections import deque
from typing import Iterable
from math import exp


class RegionalPID:
    def __init__(self, Ki: float, Kp: float, Kd: float, d_smoothing: float=0.8, i_decay:float=0.01, min_output=0, max_output=1, controllable_region=0.2):
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd
        self.d_smoothing = d_smoothing
        self.i_decay=i_decay
        self.min_output = min_output
        self.max_output=max_output

        self.last_ts = None
        self.last_state = None
        
        self.p= 0
        self.i = 0
        self.d = 0


    def report_state(self, ts: float, state: float):
        if self.last_ts is not None:
            dt = ts - self.last_ts
            dx = state - self.last_state

            self.p = dx
            self.d = (1-self.d_smoothing)*dx/dt+self.d_smoothing*self.d
            self.i += dx*dt
            self.i *= exp(-self.i_decay*dt)

        self.last_ts = ts
        self.last_state = state

    def set_controls(self, time: float, control: float):
        pass

    @property
    def output(self):
        if abs(self.p) > self.controllable_region:
            return self.min_output if self.p*self.Kp < 0 else self.max_output
        return max(
            min(
                self.Ki*self.i + self.Kd*self.d + self.Kp*self.p, self.max_output
                ),
                self.min_output)

