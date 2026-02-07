"""
Real-time motion controller with trajectory tracking
Runs in separate thread for smooth, continuous motion
"""
import threading
import time
import numpy as np
from typing import Optional, Callable

from armstack.planning.trajectory_planner import TrajectoryQueue, TrajectoryPlanner
from armstack.planning.jacobian_ik import JacobianIKSolver


class MotionController:
    """
    Real-time motion controller that streams commands to robot
    Runs in background thread for smooth trajectory tracking
    """

    def __init__(self, ik_solver: JacobianIKSolver,
                 command_callback: Callable[[np.ndarray, float], None],
                 update_rate: float = 50.0):
        """
        Args:
            ik_solver: IK solver for converting positions to joint angles
            command_callback: Function to send joint commands (angles, speed)
            update_rate: Control loop frequency (Hz)
        """
        self.ik_solver = ik_solver
        self.command_callback = command_callback
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate

        # Trajectory management
        planner = TrajectoryPlanner(max_velocity=0.8, max_acceleration=1.5)
        self.trajectory_queue = TrajectoryQueue(planner)

        # State
        self.current_joint_angles = np.zeros(3)
        self.target_ee_position = None
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()

        # Performance monitoring
        self.last_command_time = None
        self.command_count = 0

    def start(self):
        """Start the motion controller thread"""
        if self.is_running:
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        print(f"Motion controller started at {self.update_rate} Hz")

    def stop(self):
        """Stop the motion controller thread"""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        print("Motion controller stopped")

    def move_to(self, ee_position: np.ndarray, smooth: bool = True):
        """
        Command end effector to move to target position

        Args:
            ee_position: Target [x, y, z] (mm)
            smooth: If True, use trajectory planning; if False, immediate IK
        """
        with self.lock:
            self.target_ee_position = ee_position.copy()

            if smooth:
                # Solve IK for target
                success, q_target, msg = self.ik_solver.solve(
                    ee_position, self.current_joint_angles
                )

                if success:
                    # Add to trajectory queue
                    self.trajectory_queue.add_waypoint(q_target)
                else:
                    print(f"IK failed: {msg}")
            else:
                # Immediate move (for backwards compatibility)
                success, q_target, msg = self.ik_solver.solve(
                    ee_position, self.current_joint_angles
                )

                if success:
                    self.current_joint_angles = q_target
                    self._send_command(q_target)

    def move_to_with_fixed_base(self, ee_position: np.ndarray, base_angle: float):
        """
        Move to position with base angle fixed (for Z-only motion)

        Args:
            ee_position: Target [x, y, z] (mm)
            base_angle: Fixed base angle (rad)
        """
        with self.lock:
            success, q_target, msg = self.ik_solver.solve_with_base_fixed(
                ee_position, base_angle, self.current_joint_angles
            )

            if success:
                self.trajectory_queue.add_waypoint(q_target)
            else:
                print(f"IK failed: {msg}")

    def _control_loop(self):
        """Main control loop running in background thread"""
        while self.is_running:
            loop_start = time.time()

            with self.lock:
                # Get current trajectory setpoint
                setpoint = self.trajectory_queue.get_current_setpoint()

                if setpoint is not None:
                    # Update current state
                    self.current_joint_angles = setpoint.position

                    # Send command to robot
                    self._send_command(setpoint.position)

            # Sleep to maintain update rate
            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Running slow, skip sleep
                pass

    def _send_command(self, joint_angles: np.ndarray):
        """Send joint angle command to robot"""
        try:
            # Convert rad to degrees for command
            angles_deg = np.rad2deg(joint_angles)

            # Calculate commanded speed based on update rate
            # This ensures smooth motion
            speed_deg_s = 180.0 / self.update_rate  # Conservative speed

            # Call the command callback
            self.command_callback(angles_deg, speed_deg_s)

            # Update stats
            self.command_count += 1
            self.last_command_time = time.time()

        except Exception as e:
            print(f"Command send error: {e}")

    def get_current_position(self) -> np.ndarray:
        """Get current end effector position"""
        with self.lock:
            return self.ik_solver.forward_kinematics(self.current_joint_angles)

    def get_current_joints(self) -> np.ndarray:
        """Get current joint angles"""
        with self.lock:
            return self.current_joint_angles.copy()

    def set_current_joints(self, joint_angles: np.ndarray):
        """Update current joint angles (for initialization)"""
        with self.lock:
            self.current_joint_angles = joint_angles.copy()

    def is_moving(self) -> bool:
        """Check if robot is currently moving"""
        with self.lock:
            return not self.trajectory_queue.is_complete()

    def wait_for_completion(self, timeout: float = 10.0):
        """Wait for current trajectory to complete"""
        start_time = time.time()
        while self.is_moving() and (time.time() - start_time) < timeout:
            time.sleep(0.05)

    def get_stats(self) -> dict:
        """Get controller statistics"""
        return {
            'is_running': self.is_running,
            'command_count': self.command_count,
            'update_rate': self.update_rate,
            'is_moving': self.is_moving()
        }
