#!/usr/bin/env python3
"""
TF2 Transform broadcaster for 6-axis robotic arm.
Publishes the complete transform tree from /joint_states.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf_conversions import transformations
import numpy as np


class ArmTF2Broadcaster:
    """Broadcasts TF2 transforms for all arm links."""

    LINK_CHAIN = [
        ('world', 'base_link', None),
        ('base_link', 'link_1', 'joint_1'),
        ('link_1', 'link_2', 'joint_2'),
        ('link_2', 'link_3', 'joint_3'),
        ('link_3', 'link_4', 'joint_4'),
        ('link_4', 'link_5', 'joint_5'),
        ('link_5', 'link_6', 'joint_6'),
        ('link_6', 'tool_tip', None),
    ]

    JOINT_ORIGINS = {
        'world_to_base': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
        'joint_1': {'xyz': [0, 0, 0.10], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
        'joint_2': {'xyz': [0, 0, 0.15], 'rpy': [np.pi/2, 0, 0], 'axis': [0, 0, 1]},
        'joint_3': {'xyz': [0.30, 0, 0], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
        'joint_4': {'xyz': [0.25, 0, 0], 'rpy': [np.pi/2, 0, 0], 'axis': [0, 0, 1]},
        'joint_5': {'xyz': [0, 0, 0.20], 'rpy': [-np.pi/2, 0, 0], 'axis': [0, 0, 1]},
        'joint_6': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
        'tool_tip_joint': {'xyz': [0, 0, 0.10], 'rpy': [0, 0, 0]},
    }

    def __init__(self):
        rospy.init_node('arm_tf2_broadcaster')
        self.br = tf2_ros.TransformBroadcaster()
        self.joint_states = None
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.rate = rospy.Rate(50)

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def broadcast_transforms(self):
        if self.joint_states is None:
            return

        joint_dict = {}
        for name, position in zip(self.joint_states.name, self.joint_states.position):
            joint_dict[name] = position

        transforms = []
        timestamp = rospy.Time.now()

        for parent, child, joint_name in self.LINK_CHAIN:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent
            t.child_frame_id = child

            if joint_name is None:
                if parent == 'world' and child == 'base_link':
                    origin = self.JOINT_ORIGINS['world_to_base']
                else:
                    origin = self.JOINT_ORIGINS['tool_tip_joint']
            else:
                origin = self.JOINT_ORIGINS[joint_name]

            xyz = origin['xyz']
            rpy = origin['rpy']

            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])

            quat = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            if joint_name and joint_name in joint_dict:
                angle = joint_dict[joint_name]
                axis = origin['axis']
                quat_rot = transformations.quaternion_about_axis(angle, axis)
                base_quat = np.array([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                ])
                combined = transformations.quaternion_multiply(base_quat, quat_rot)
                t.transform.rotation.x = combined[0]
                t.transform.rotation.y = combined[1]
                t.transform.rotation.z = combined[2]
                t.transform.rotation.w = combined[3]

            transforms.append(t)

        self.br.sendTransform(transforms)

    def run(self):
        while not rospy.is_shutdown():
            self.broadcast_transforms()
            self.rate.sleep()


def main():
    try:
        broadcaster = ArmTF2Broadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
