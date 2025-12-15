import math

class CollisionMonitor:
    def __init__(self, stop_dist=0.35):
        self.stop_dist = stop_dist
        self.min_range = math.inf

    def update_scan(self, scan_msg):
        vals = [r for r in scan_msg.ranges if scan_msg.range_min < r < scan_msg.range_max]
        self.min_range = min(vals) if vals else math.inf

    def is_safe(self):
        return self.min_range > self.stop_dist, self.min_range
