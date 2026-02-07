"""
Jacobian-based inverse kinematics solver
More robust than analytical IK, handles singularities better
"""
import numpy as np
from typing import Tuple, Optional


class JacobianIKSolver:
    """
    Iterative IK solver using Jacobian pseudo-inverse
    Handles joint limits and singularities
    """

    def __init__(self, L1: float, L2: float):
        """
        Args:
            L1: Length of first link (mm)
            L2: Length of second link (mm)
        """
        self.L1 = L1
        self.L2 = L2

        # Joint limits (rad)
        self.q_min = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
        self.q_max = np.array([np.pi/2, np.pi/2, np.pi/2])

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """
        Compute end effector position from joint angles

        Args:
            q: Joint angles [base, shoulder, elbow] (rad)

        Returns:
            End effector position [x, y, z] (mm)
        """
        base, shoulder, elbow = q

        # 2D arm in vertical plane
        # Add pi/2 to shoulder since 0 is vertical
        theta1 = shoulder + np.pi/2
        theta2 = elbow

        # Position in vertical plane
        r = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        z = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)

        # Rotate by base angle to get 3D position
        x = r * np.cos(base)
        y = r * np.sin(base)

        return np.array([x, y, z])

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix (3x3) relating joint velocities to EE velocities

        Args:
            q: Joint angles [base, shoulder, elbow] (rad)

        Returns:
            Jacobian matrix (3x3)
        """
        base, shoulder, elbow = q

        theta1 = shoulder + np.pi/2
        theta2 = elbow

        # Intermediate calculations
        r = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        c_base = np.cos(base)
        s_base = np.sin(base)

        # dr/dtheta (derivatives of r wrt shoulder and elbow)
        dr_dtheta1 = -self.L1 * np.sin(theta1) - self.L2 * np.sin(theta1 + theta2)
        dr_dtheta2 = -self.L2 * np.sin(theta1 + theta2)

        # dz/dtheta
        dz_dtheta1 = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        dz_dtheta2 = self.L2 * np.cos(theta1 + theta2)

        # Jacobian matrix
        J = np.array([
            [-r * s_base, dr_dtheta1 * c_base, dr_dtheta2 * c_base],  # dx/dq
            [r * c_base,  dr_dtheta1 * s_base, dr_dtheta2 * s_base],   # dy/dq
            [0.0,         dz_dtheta1,          dz_dtheta2]              # dz/dq
        ])

        return J

    def solve(self, target: np.ndarray, q_init: Optional[np.ndarray] = None,
              max_iter: int = 100, tol: float = 1.0) -> Tuple[bool, np.ndarray, str]:
        """
        Solve IK using Jacobian pseudo-inverse method

        Args:
            target: Target position [x, y, z] (mm)
            q_init: Initial joint angles (rad), if None uses zero
            max_iter: Maximum iterations
            tol: Position tolerance (mm)

        Returns:
            (success, joint_angles, message)
        """
        # Initialize
        if q_init is None:
            q = np.zeros(3)
        else:
            q = q_init.copy()

        # Clamp to joint limits
        q = np.clip(q, self.q_min, self.q_max)

        # Iterative IK
        alpha = 0.5  # Step size (damping factor)
        lambda_damping = 0.01  # Damping for pseudo-inverse

        for iteration in range(max_iter):
            # Forward kinematics
            current_pos = self.forward_kinematics(q)

            # Error
            error = target - current_pos
            error_norm = np.linalg.norm(error)

            # Check convergence
            if error_norm < tol:
                return True, q, "Converged"

            # Compute Jacobian
            J = self.compute_jacobian(q)

            # Damped pseudo-inverse (handles singularities better)
            JTJ = J.T @ J + lambda_damping * np.eye(3)
            try:
                J_pinv = np.linalg.solve(JTJ, J.T)
            except np.linalg.LinAlgError:
                return False, q, "Singular configuration"

            # Compute joint update
            dq = alpha * (J_pinv @ error)

            # Update joints with limits
            q_new = q + dq
            q_new = np.clip(q_new, self.q_min, self.q_max)

            # Check for joint limit violations causing no progress
            if np.linalg.norm(q_new - q) < 1e-6:
                if error_norm > tol:
                    return False, q, "Joint limits prevent reaching target"
                break

            q = q_new

        # Check if we got close enough
        final_pos = self.forward_kinematics(q)
        final_error = np.linalg.norm(target - final_pos)

        if final_error < tol * 2:  # Allow 2x tolerance
            return True, q, "Reached near target"

        return False, q, f"Did not converge (error: {final_error:.1f}mm)"

    def solve_with_base_fixed(self, target: np.ndarray, base_angle: float,
                              q_init: Optional[np.ndarray] = None,
                              **kwargs) -> Tuple[bool, np.ndarray, str]:
        """
        Solve IK with base angle fixed (for Z-only motion)

        Args:
            target: Target position [x, y, z] (mm)
            base_angle: Fixed base angle (rad)
            q_init: Initial joint angles (rad)
            **kwargs: Additional arguments for solve()

        Returns:
            (success, joint_angles, message)
        """
        if q_init is None:
            q_init = np.array([base_angle, 0.0, 0.0])
        else:
            q_init = q_init.copy()
            q_init[0] = base_angle

        success, q, msg = self.solve(target, q_init, **kwargs)

        # Force base angle to stay fixed
        q[0] = base_angle

        return success, q, msg
