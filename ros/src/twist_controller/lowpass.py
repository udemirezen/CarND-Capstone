
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
            
        else:
            self.ready = True

        self.last_val = val
        return val

class SmoothingFilter(object):
    def __init__(self, w_weight):
        self.last_value = 0
        self.w_weight = w_weight

    def get_smoothed_value(self, value):
        self.last_value = (self.w_weight * self.last_value) + ((1.0 - self.w_weight) * value)
        return self.last_value
