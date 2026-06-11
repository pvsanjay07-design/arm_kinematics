#!/usr/bin/env python3

import numpy as np
from scipy.optimize import minimize
import rospy
import os
import yaml
import rospkg


class IKSolver:
    def __init__(self):
        # DH Parameters: [a, alpha, d, theta_offset]
        # Based on a typical 6-axis industrial arm geometry
        self.dh_params = np.array([
            [0.0,    np.pi/2,  0.1,   0.0],   # Joint 1
            [0.3,    0.0,      0.0,   0.0],   # Joint 2
            [0.25,   0.0,      0.0,   0.0],   # Joint 3
            [0.0,    np.pi/2,  0.2,   0.0],   # Joint 4
            [0.0,   -np.pi/2,  0.0,   0.0],   # Joint 5
            [0.0,    0.0,      0.08,  0.0],   # Joint 6
        ])

        # Joint limits [min, max] in radians
        self.joint_limits = [
            [-np.pi,      np.pi    ],  # joint_1
            [-np.pi/2,    np.pi/2  ],  # joint_2
            [-np.pi/2,    np.pi/2  ],  # joint_3
            [-np.pi,      np.pi    ],  # joint_4
            [-np.pi/2,    np.pi/2  ],  # joint_5
            [-np.pi,      np.pi    ],  # joint_6
        ]

        # Try loading from yaml if available
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('arm_kinematics')
            yaml_path = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            limits = data.get('joint_limits', {})
            loaded = []
            for i in range(1, 7):
                jname = f'joint_{i}'
                if jname in limits:
                    loaded.append([limits[jname]['min'], limits[jname]['max']])
            if len(loaded) == 6:
                self.joint_limits = loaded
        except Exception:
            pass  # Use defaults defined above

        self.max_iterations = 1000
        self.tolerance = 1e-4
        self.step_size = 0.1
        rospy.loginfo("IK Solver initialized with DH parameters.")

    def dh_transform(self, a, alpha, d, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct,  -st*ca,   st*sa,  a*ct],
            [st,   ct*ca,  -ct*sa,  a*st],
            [0.0,  sa,      ca,     d   ],
            [0.0,  0.0,     0.0,    1.0 ]
        ])

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i in range(6):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = joint_angles[i] + theta_offset
            T = T @ self.dh_transform(a, alpha, d, theta)
        return T

    def compute_jacobian(self, joint_angles):
        delta = 1e-6
        J = np.zeros((3, 6))
        T_base = self.forward_kinematics(joint_angles)
        pos_base = T_base[:3, 3]
        for i in range(6):
            angles_plus = list(joint_angles)
            angles_minus = list(joint_angles)
            angles_plus[i] += delta
            angles_minus[i] -= delta
            T_plus = self.forward_kinematics(angles_plus)
            T_minus = self.forward_kinematics(angles_minus)
            J[:, i] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * delta)
        return J

    def solve_ik(self, target_position, target_orientation):
        target_pos = np.array(target_position)

        # --- STEP 1: Jacobian pseudoinverse attempt ---
        joint_angles = np.zeros(6)
        for iteration in range(self.max_iterations):
            T_current = self.forward_kinematics(joint_angles)
            current_pos = T_current[:3, 3]
            error = target_pos - current_pos
            if np.linalg.norm(error) < self.tolerance:
                rospy.loginfo(f"IK converged in {iteration} iterations.")
                return joint_angles.tolist()
            J = self.compute_jacobian(joint_angles)
            J_pinv = np.linalg.pinv(J)
            delta_q = J_pinv @ error * self.step_size
            joint_angles = joint_angles + delta_q
            # Clamp to joint limits
            for i in range(6):
                joint_angles[i] = np.clip(
                    joint_angles[i],
                    self.joint_limits[i][0],
                    self.joint_limits[i][1]
                )

        rospy.logwarn("Jacobian IK did not converge. Trying SQP fallback...")

        # --- STEP 2: SQP fallback ---
        def objective(q):
            T = self.forward_kinematics(q)
            pos = T[:3, 3]
            return np.linalg.norm(target_pos - pos) ** 2

        bounds = [(lim[0], lim[1]) for lim in self.joint_limits]
        rng = np.random.default_rng(42)
        best_result = None
        best_error = float('inf')

        for _ in range(5):
            x0 = rng.uniform(
                [b[0] for b in bounds],
                [b[1] for b in bounds]
            )
            result = minimize(
                objective, x0,
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 1000, 'ftol': 1e-8}
            )
            if result.fun < best_error:
                best_error = result.fun
                best_result = result

        if best_result is not None and best_error < 1e-4:
            rospy.loginfo("IK solved via SQP fallback.")
            return best_result.x.tolist()

        raise ValueError(
            f"IK failed to converge for target position {target_position}. "
            f"Best error: {best_error:.6f}. Target may be out of reach."
        )
