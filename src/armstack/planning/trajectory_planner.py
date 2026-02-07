"""
Trajectory planner for smooth robot motion
Generates time-parameterized paths with velocity/acceleration profiles
"""
import numpy as np
from typing import List, Tuple
import time


class TrajectoryPoint:
    """Single point in a trajectory"""
    def __init__(self, position: np.ndarray, velocity: np.ndarray,
                 acceleration: np.ndarray, time: float):
        self.position = position  # Joint angles (rad)
        self.velocity = velocity  # Joint velocities (rad/s)
        self.acceleration = acceleration  # Joint accelerations (rad/s^2)
        self.time = time  # Time from trajectory start (s)


class TrajectoryPlanner:
    """
    Generates smooth trajectories between waypoints using cubic polynomials
    with trapezoidal velocity profiles
    """

    def __init__(self, max_velocity: float = 1.0, max_acceleration: float = 2.0):
        """
        Args:
            max_velocity: Maximum joint velocity (rad/s)
            max_acceleration: Maximum joint acceleration (rad/s^2)
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

    def plan_trajectory(self, waypoints: List[np.ndarray], dt: float = 0.05) -> List[TrajectoryPoint]:
        """
        Generate a smooth trajectory through waypoints

        Args:
            waypoints: List of joint angle arrays [q1, q2, q3] (rad)
            dt: Time step for discretization (s)

        Returns:
            List of TrajectoryPoint objects
        """
        if len(waypoints) < 2:
            return []

        trajectory = []
        current_time = 0.0

        # Add initial point at rest
        trajectory.append(TrajectoryPoint(
            position=waypoints[0],
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
            time=0.0
        ))

        # Generate segments between waypoints
        for i in range(len(waypoints) - 1):
            segment_points = self._generate_segment(
                waypoints[i], waypoints[i+1], current_time, dt
            )
            trajectory.extend(segment_points)
            if segment_points:
                current_time = segment_points[-1].time

        return trajectory

    def _generate_segment(self, start: np.ndarray, end: np.ndarray,
                         start_time: float, dt: float) -> List[TrajectoryPoint]:
        """Generate trajectory segment between two waypoints"""

        # Calculate distance in joint space
        delta = end - start
        distance = np.linalg.norm(delta)

        if distance < 1e-6:
            return []

        # Calculate segment duration based on trapezoidal velocity profile
        # T = distance / avg_velocity, with acceleration phase
        t_accel = self.max_velocity / self.max_acceleration
        d_accel = 0.5 * self.max_acceleration * t_accel**2

        if distance < 2 * d_accel:
            # Triangular profile (no constant velocity phase)
            t_total = 2 * np.sqrt(distance / self.max_acceleration)
            t_accel = t_total / 2
        else:
            # Trapezoidal profile
            t_const = (distance - 2 * d_accel) / self.max_velocity
            t_total = 2 * t_accel + t_const

        # Generate discrete points along trajectory
        num_points = int(t_total / dt)
        if num_points < 2:
            num_points = 2

        points = []
        direction = delta / distance

        for i in range(1, num_points + 1):
            t = i * dt
            if t > t_total:
                t = t_total

            # Calculate position, velocity, acceleration using trapezoidal profile
            if t <= t_accel:
                # Acceleration phase
                s = 0.5 * self.max_acceleration * t**2
                v = self.max_acceleration * t
                a = self.max_acceleration
            elif t <= (t_total - t_accel):
                # Constant velocity phase
                s = d_accel + self.max_velocity * (t - t_accel)
                v = self.max_velocity
                a = 0.0
            else:
                # Deceleration phase
                t_dec = t - (t_total - t_accel)
                s = distance - 0.5 * self.max_acceleration * (t_accel - t_dec)**2
                v = self.max_acceleration * (t_accel - t_dec)
                a = -self.max_acceleration

            # Scale to distance
            s = min(s, distance)

            position = start + direction * s
            velocity = direction * v
            acceleration = direction * a

            points.append(TrajectoryPoint(
                position=position,
                velocity=velocity,
                acceleration=acceleration,
                time=start_time + t
            ))

        return points


class TrajectoryQueue:
    """Thread-safe queue for managing trajectory waypoints"""

    def __init__(self, planner: TrajectoryPlanner):
        self.planner = planner
        self.waypoints = []
        self.current_trajectory = []
        self.current_index = 0
        self.last_update_time = None

    def add_waypoint(self, joint_angles: np.ndarray):
        """Add a waypoint to the queue"""
        self.waypoints.append(joint_angles.copy())

        # If this is the first waypoint or we have enough, generate trajectory
        if len(self.waypoints) >= 2:
            self._regenerate_trajectory()

    def _regenerate_trajectory(self):
        """Regenerate trajectory from current waypoints"""
        if len(self.waypoints) < 2:
            return

        # Keep only the last few waypoints to avoid lag
        if len(self.waypoints) > 5:
            self.waypoints = self.waypoints[-3:]

        self.current_trajectory = self.planner.plan_trajectory(self.waypoints)
        self.current_index = 0
        self.last_update_time = time.time()

    def get_current_setpoint(self) -> TrajectoryPoint:
        """Get the current trajectory setpoint"""
        if not self.current_trajectory:
            return None

        current_time = time.time()
        if self.last_update_time is None:
            self.last_update_time = current_time

        elapsed = current_time - self.last_update_time

        # Find appropriate point based on elapsed time
        while (self.current_index < len(self.current_trajectory) - 1 and
               self.current_trajectory[self.current_index].time < elapsed):
            self.current_index += 1

        if self.current_index >= len(self.current_trajectory):
            # Trajectory complete, clear waypoints
            self.waypoints = [self.waypoints[-1]] if self.waypoints else []
            return None

        return self.current_trajectory[self.current_index]

    def is_complete(self) -> bool:
        """Check if trajectory is complete"""
        return (not self.current_trajectory or
                self.current_index >= len(self.current_trajectory) - 1)
