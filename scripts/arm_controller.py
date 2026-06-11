#!/usr/bin/env python3

import rospy
import sys
import os

# Allow importing sibling scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from inverse_kinematics import IKSolver
from trajectory_planner import TrajectoryPlanner

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf.transformations


class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=False)

        self.ik_solver   = IKSolver()
        self.planner     = TrajectoryPlanner()

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        self.current_joint_positions = [0.0] * 6

        # Publishers
        self.traj_pub = rospy.Publisher(
            '/arm_position_controller/command',
            JointTrajectory,
            queue_size=10
        )
        self.joint_state_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10
        )

        # Subscribers
        rospy.Subscriber('/joint_states', JointState,
                         self._joint_state_cb)
        rospy.Subscriber('/target_pose', Pose,
                         self._pose_callback)

        # Give publishers time to connect
        rospy.sleep(1.0)

        # Publish home position immediately for RViz
        self._publish_home()

        rospy.loginfo("Arm Controller ready. Waiting for /target_pose...")

    def _joint_state_cb(self, msg):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        for i, jname in enumerate(self.joint_names):
            if jname in name_to_idx:
                self.current_joint_positions[i] = \
                    msg.position[name_to_idx[jname]]

    def _publish_home(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name     = self.joint_names
        js.position = [0.0] * 6
        js.velocity = [0.0] * 6
        js.effort   = [0.0] * 6
        self.joint_state_pub.publish(js)
        rospy.loginfo("Published home joint state.")

    def _pose_callback(self, msg):
        rospy.loginfo(
            f"Received target pose: "
            f"x={msg.position.x:.3f}, "
            f"y={msg.position.y:.3f}, "
            f"z={msg.position.z:.3f}"
        )

        # --- STEP 1: Extract target ---
        target_position = [
            msg.position.x,
            msg.position.y,
            msg.position.z
        ]
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = tf.transformations.euler_from_quaternion(quat)
        target_orientation = list(euler)  # [roll, pitch, yaw]

        # --- STEP 2: Solve IK ---
        try:
            target_joints = self.ik_solver.solve_ik(
                target_position, target_orientation
            )
        except ValueError as e:
            rospy.logerr(f"IK failed: {e}")
            return

        rospy.loginfo(
            f"IK solution: "
            + ", ".join([f"j{i+1}={v:.3f}" for i, v in enumerate(target_joints)])
        )

        # --- STEP 3: Get start joints ---
        start_joints = list(self.current_joint_positions)

        # --- STEP 4: Plan trajectory ---
        waypoints = self.planner.generate_trajectory(
            start_joints, target_joints,
            duration=5.0, num_points=50
        )

        # --- STEP 5: Build JointTrajectory message ---
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names  = self.joint_names

        for wp in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions     = wp['positions']
            pt.velocities    = wp['velocities']
            pt.accelerations = wp['accelerations']
            pt.time_from_start = rospy.Duration(wp['time_from_start'])
            traj_msg.points.append(pt)

        # --- STEP 6: Publish ---
        self.traj_pub.publish(traj_msg)
        rospy.loginfo(
            f"Trajectory published: {len(waypoints)} waypoints "
            f"over {waypoints[-1]['time_from_start']:.1f}s."
        )

        # Also publish final joint state for RViz fallback
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name     = self.joint_names
        js.position = target_joints
        js.velocity = [0.0] * 6
        js.effort   = [0.0] * 6
        self.joint_state_pub.publish(js)


if __name__ == '__main__':
    try:
        controller = ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
