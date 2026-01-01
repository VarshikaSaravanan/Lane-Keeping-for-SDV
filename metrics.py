
import numpy as np
import collections

class MetricTracker:
    def __init__(self, history_length=50):
        """
        Tracks data for plotting.
        """
        self.history_length = history_length
        self.steering_history = collections.deque(maxlen=history_length)
        self.jerk_history = collections.deque(maxlen=history_length)
        self.prev_steering = 0.0

    def update(self, steering_angle):
        # Calculate instantaneous "Jerk" (simulated as delta steering)
        # Real Jerk is d(Accel)/dt, but steering rate is a good proxy for comfort here.
        delta = steering_angle - self.prev_steering
        self.prev_steering = steering_angle
        
        self.steering_history.append(steering_angle)
        self.jerk_history.append(abs(delta)) # Magnitude of change
        
    def get_data(self):
        return list(self.steering_history), list(self.jerk_history)
