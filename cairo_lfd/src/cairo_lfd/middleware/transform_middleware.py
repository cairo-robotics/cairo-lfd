import time

import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import inverse_matrix
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3, Pose, PoseStamped

'''
Helper functions modified from https://github.com/cu-ironlab/ros_arvr_device_and_robot_management/blob/master/src/arvr_utility/msg_transformations.py
'''


def target_to_world(pose, transformation_matrix):
    """
    This is a helper function to transform a target coordinate to the world space when given the
    selector transformation matrix that is specific to that arvr device.
    :type pose: Pose
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    translation_vector = np.array([pose.position.x,
                                   pose.position.y,
                                   pose.position.z,
                                   1]).transpose()
    quaternion_vector = np.array([pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z,
                                  pose.orientation.w]).transpose()
    translation_vector = transformation_matrix.dot(translation_vector)
    quaternion_vector = transformation_matrix.dot(quaternion_vector)

    p = Pose()
    p.position.x = translation_vector[0]
    p.position.y = translation_vector[1]
    p.position.z = translation_vector[2]
    p.orientation.x = quaternion_vector[0]
    p.orientation.y = quaternion_vector[1]
    p.orientation.z = quaternion_vector[2]
    p.orientation.w = quaternion_vector[3]

    return p


def world_to_target(pose, transformation_matrix):
    """
    This is a helper function to transform a world coordinate to an arvr coordinate when given the
    selector transformation matrix that is specific to that arvr device. To convert it back in the
    opposite direction, you must use the inverse of the transformation_matrix and then do the normal
    swaps in the arvr_to_world() function.
    :type pose: Pose
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    inverse = inverse_matrix(transformation_matrix)
    return target_to_world(pose, inverse)


'''
Contains a class for transforming from a fixed coordinate system to a HoloLens with a known origin point

Code modified from Dan Koral's arvr_device package: https://github.com/cu-ironlab/ros_arvr_device_and_robot_management
'''


class ARVRFixedTransform(object):
    def __init__(self, world_frame, child_frame, origin_translation, origin_rotation, selector_matrix):
        """
        :type child_frame: str
        :type world_frame: str
        :type origin_translation: Vector3
        :type origin_rotation: Quaternion
        :type selector_matrix: list
        :param name:
        :param origin_translation:
        :param origin_rotation:
        :param selector_matrix:
        """
        self.child_frame = child_frame
        self.world_frame = world_frame
        self.selector_matrix = np.array(selector_matrix)
        self.last_lookup = time.time()
        self.last_inverse_lookup = None
        self.frame_transform = None
        self.inverse_transform = None
        self._update_transform(origin_translation, origin_rotation)

        # TF Specific Stuff
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        self._publish_transform()

    def _update_transform(self, translation, rotation):
        """
        :type translation: Point
        :type rotation: Quaternion
        :param translation:
        :param rotation:
        """
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = self.world_frame
        self.static_transform.child_frame_id = self.child_frame
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)

    def world_to_target(self, pose):
        """
        :type pose: Pose
        :param pose: point in world coordinates to be transformed to target space
        """

        # lookup transform between world and this device if it has been longer than one second since last lookup
        if(self.last_lookup is None or time.time() - self.last_lookup > 1):
            self.frame_transform = self.tf2_buffer.lookup_transform(
                self.name, "world", rospy.Time(0), rospy.Duration(1.0))
            self.last_lookup = time.time()

        # apply transform to point
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(
            pose_stamped, self.frame_transform)

        # switch coordinate axes
        device_pose = world_to_target(transformed_pose.pose, self.selector_matrix)
        pose_stamped.pose = device_pose
        return pose_stamped

    def target_to_world(self, pose):
        """
        :type pose: Pose
        :param pose: point in target coordinates to be transformed to world space
        """
        # lookup transform between this device and world if it has been longer than one second since last lookup
        if(self.last_inverse_lookup is None or time.time() - self.last_inverse_lookup > 1):
            self.inverse_transform = self.tf2_buffer.lookup_transform(
                "world", self.name, rospy.Time(0), rospy.Duration(1.0))
            self.last_inverse_lookup = time.time()
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.name
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose

        # switch coordinate axes
        pose_stamped.pose = target_to_world(pose_stamped.pose, self.selector_matrix)

        # apply transform to point
        transformed_pose = tf2_geometry_msgs.do_transform_pose(
            pose_stamped, self.inverse_transform)

        return transformed_pose



class FixedTransform():
    def __init__(self, parent_frame="world", child_frame="optitrack_world", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.update_transform(translation, rotation)

        # TF Specific Stuff
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        
        self._publish_transform()

    def update_transform(self, translation, rotation):
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = self.parent_frame
        self.static_transform.child_frame_id = self.child_frame
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)


