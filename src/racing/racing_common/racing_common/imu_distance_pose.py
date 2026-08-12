"""Position integration using odometry distance and an IMU heading."""

import math
from typing import Optional, Tuple


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ImuDistancePose:
    """Integrate scalar `/odom_combined` travel in the supplied IMU yaw frame."""

    def __init__(self, max_step_m: float = 0.12, min_step_m: float = 0.0):
        self.max_step_m = max(0.02, max_step_m)
        self.min_step_m = max(0.0, min(min_step_m, self.max_step_m))
        self.pose: Optional[Tuple[float, float]] = None
        self.total_distance_m = 0.0
        self._last_position: Optional[Tuple[float, float]] = None
        self._last_yaw: Optional[float] = None

    def reset(self, odom_position: Tuple[float, float], yaw: float) -> None:
        self.pose = (0.0, 0.0)
        self.total_distance_m = 0.0
        self._last_position = odom_position
        self._last_yaw = yaw

    def update(self, odom_position: Tuple[float, float], yaw: float,
               direction: float = 1.0) -> Tuple[float, float]:
        if self.pose is None or self._last_position is None or self._last_yaw is None:
            self.reset(odom_position, yaw)
            return self.pose
        step = math.hypot(odom_position[0] - self._last_position[0],
                          odom_position[1] - self._last_position[1])
        mid_yaw = _wrap_angle(self._last_yaw + 0.5 * _wrap_angle(yaw - self._last_yaw))
        self._last_position = odom_position
        self._last_yaw = yaw
        if step < self.min_step_m or step > self.max_step_m:
            return self.pose
        step = math.copysign(step, direction if direction else 1.0)
        self.pose = (self.pose[0] + step * math.cos(mid_yaw),
                     self.pose[1] + step * math.sin(mid_yaw))
        self.total_distance_m += abs(step)
        return self.pose
