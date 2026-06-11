#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import CubicSpline
import rospy
import os
import yaml
import rospkg


class TrajectoryPlanner:
    def __init__(self):
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        self.joint_limits = {
            'joint_1': {'min': -3.14159, 'max': 3.14159},
            'joint_2': {'min': -1.5708,  'max': 1.5708 },
            'joint_3': {'min': -1.5708,  'max': 1.5708 },
            'joint_4': {'min': -3.14159, 'max': 3.14159},
            'joint_5': {'min': -1.5708,  'max': 1.5708 },
            'joint_6': {'min': -3.14159, 'max': 3.14159},
        }
        self._load_joint_limits()
        rospy.loginfo("Trajectory Planner initialized.")

    def _load_joint_limits(self):
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('arm_kinematics')
            yaml_path = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            limits = data.get('joint_limits', {})
            for jname in self.joint_names:
                if jname in limits:
                    self.joint_limits[jname] = limits[jname]
        except Exception as e:
            rospy.logwarn(f"Could not load joint_limits.yaml: {e}. Using defaults.")

    def generate_trajectory(self, start_joints, end_joints,
                            duration=5.0, num_points=50):
        start = np.array(start_joints)
        end   = np.array(end_joints)

        # Time array
        t_array = np.linspace(0.0, duration, num_points)
        t_key   = np.array([0.0, duration])

        waypoints = []

        # Build spline per joint
        splines = []
        for i in range(6):
            cs = CubicSpline(t_key, [start[i], end[i]],
                             bc_type='clamped')
            splines.append(cs)

        for k, t in enumerate(t_array):
            positions     = []
            velocities    = []
            accelerations = []

            for i in range(6):
                pos  = float(splines[i](t))
                vel  = float(splines[i](t, 1))
                acc  = float(splines[i](t, 2))

                # Clamp to joint limits
                jname = self.joint_names[i]
                jmin  = self.joint_limits[jname]['min']
                jmax  = self.joint_limits[jname]['max']
                if pos < jmin or pos > jmax:
                    rospy.logwarn(
                        f"Waypoint {k} joint {jname} = {pos:.4f} "
                        f"clamped to [{jmin}, {jmax}]"
                    )
                    pos = float(np.clip(pos, jmin, jmax))

                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            waypoints.append({
                'positions':     positions,
                'velocities':    velocities,
                'accelerations': accelerations,
                'time_from_start': float(t)
            })

        rospy.loginfo(
            f"Trajectory planned: {num_points} waypoints over {duration}s."
        )
        return waypoints
