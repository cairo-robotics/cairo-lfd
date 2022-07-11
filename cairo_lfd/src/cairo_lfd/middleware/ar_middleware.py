import rospy
import tf2_ros
import tf2_geometry_msgs
import json
import time
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Transform, Pose, PoseStamped, Point, Quaternion, Vector3
from tf.transformations import inverse_matrix

from collision_ik.msg import EEPoseGoals, JointAngles


'''
Helper functions modified from https://github.com/cu-ironlab/ros_arvr_device_and_robot_management/blob/master/src/arvr_utility/msg_transformations.py
'''


def arvr_to_world(msg, transformation_matrix):
    """
    This is a helper function to transform a arvr coordinate to the world space when given the
    selector transformation matrix that is specific to that arvr device.
    :type msg: PoseStamped
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    translation_vector = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z,
                                   1]).transpose()
    quaternion_vector = np.array([msg.pose.orientation.x,
                                  msg.pose.orientation.y,
                                  msg.pose.orientation.z,
                                  msg.pose.orientation.w]).transpose()

    translation_vector = transformation_matrix.dot(translation_vector)
    quaternion_vector = transformation_matrix.dot(quaternion_vector)

    p = PoseStamped()
    p.header = msg.header
    p.pose.position.x = translation_vector[0]
    p.pose.position.y = translation_vector[1]
    p.pose.position.z = translation_vector[2]
    p.pose.orientation.x = quaternion_vector[0]
    p.pose.orientation.y = quaternion_vector[1]
    p.pose.orientation.z = quaternion_vector[2]
    p.pose.orientation.w = quaternion_vector[3]

    return p


def world_to_arvr(msg, transformation_matrix):
    """
    This is a helper function to transform a world coordinate to an arvr coordinate when given the
    selector transformation matrix that is specific to that arvr device. To convert it back in the
    opposite direction, you must use the inverse of the transformation_matrix and then do the normal
    swaps in the arvr_to_world() function.
    :type msg: PoseStamped
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    inverse = inverse_matrix(transformation_matrix)
    return arvr_to_world(msg, inverse)


'''
Contains a class for transforming from a fixed coordinate system to a HoloLens with a known origin point

Code modified from Dan Koral's arvr_device package: https://github.com/cu-ironlab/ros_arvr_device_and_robot_management
'''


class ARVRFixedTransform(object):
    def __init__(self, name, origin_translation, origin_rotation, selector_matrix):
        """
        :type name: str
        :type origin_translation: Vector3
        :type origin_rotation: Quaternion
        :type selector_matrix: list
        :param name:
        :param origin_translation:
        :param origin_rotation:
        :param selector_matrix:
        """
        self.name = name
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
        self.static_transform.header.frame_id = "world"
        self.static_transform.child_frame_id = self.name
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)

    def world_to_hololens(self, pose):
        """
        :type pose: Pose
        :param pose: point in world coordinates to be transformed to HoloLens space
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
        device_pose = world_to_arvr(transformed_pose, self.selector_matrix)
        return device_pose

    def hololens_to_world(self, pose):
        """
        :type pose: Pose
        :param pose: point in HoloLens coordinates to be transformed to world space
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
        world_pose = arvr_to_world(pose_stamped, self.selector_matrix)

        # apply transform to point
        transformed_pose = tf2_geometry_msgs.do_transform_pose(
            world_pose, self.inverse_transform)

        return transformed_pose


def remap_constraints_for_lfd(json_msg):

    def height_constraint_below(constraint_id, args):
        return {
            "class": "PlanarConstraint",
            "init_args":
            {
                "constraint_id": int(constraint_id),
                "item_id": 1,
                "reference_position": float(args[0]),
                "threshold_distance": float(args[1]),
                "direction": "negative",
                "axis": "z"
            }
        }

    def height_constraint_above(constraint_id, args):
        return {
            "class": "PlanarConstraint",
            "init_args":
            {
                "constraint_id": int(constraint_id),
                "item_id": 1,
                "reference_position": float(args[0]),
                "threshold_distance": float(args[1]),
                "direction": "positive",
                "axis": "z"
            }
        }

    def upright_constraint(constraint_id, args):
        return {
            "class": "OrientationConstraint",
            "init_args":
                {
                    "constraint_id": int(constraint_id),
                    "item_id": 1,
                    "threshold_angle": float(args[4]),
                    "reference_orientation": [float(arg) for arg in args[0:4]],
                    "axis": "z"
                }
        }

    def over_under_constraint(constraint_id, args):
        return {
            "class": "OverUnderConstraint",
            "init_args":
                {
                    "constraint_id": int(constraint_id),
                    "above_item_id": 1,
                    "below_item_id": 2,
                    "threshold_distance": float(args[3]),
                    "reference_pose": {
                        "position": [float(arg) for arg in args[0:3]],
                        "orientation": [0, 0, 0, 1.0]
                    },
                    "axis": "y"
                }
        }

    mapping_functions = {
        "HeightConstraintBelow": height_constraint_below,
        "HeightConstraintAbove": height_constraint_above,
        "UprightConstraint": upright_constraint,
        "OverUnderConstraint": over_under_constraint
    }

    ar_constraints = json_msg["constraints"]

    lfd_constraints = []

    for constraint in ar_constraints:
        class_name = constraint["className"]
        cst_func = mapping_functions[class_name]
        lfd_constraints.append(cst_func(constraint["id"], constraint["args"]))

    return lfd_constraints


class AR4LfDMiddleware(object):

    def __init__(self, command_topic, request_topic, traj_representation_topic, hololens_pub_topic, constraint_edits_in, constraint_edits_out):
        """
        :type command_topic: str
        :type request_topic: str
        :type traj_representation_topic: str
        :type hololens_pub_topic: str
        :param command_topic:
        :param request_topic:
        :param traj_representation_topic:
        :param hololens_pub_topic:
        """

        # Create transform manager (position and axes hardcoded for now)
        self.transform_manager = ARVRFixedTransform("hololens", Vector3(0.975, -0.09, -0.27),
                                                    Quaternion(0.0, 0.0, 1.0, 0.0), [[0, 0, 1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, -1]])

        # Initialize Publishers
        self.request_pub = rospy.Publisher(request_topic, String, queue_size=1)
        self.hololens_pub = rospy.Publisher(
            hololens_pub_topic, String, queue_size=1)
        self.constraints_pub = rospy.Publisher(
            constraint_edits_out, String, queue_size=1)

        # Initialize Subscribers
        self.command_sub = rospy.Subscriber(
            command_topic, String, self._command_cb)
        self.trajectory_sub = rospy.Subscriber(
            traj_representation_topic, String, self._trajectory_cb)
        self.constraints_sub = rospy.Subscriber(
            constraint_edits_in, String, self._constraint_cb)

    def _command_cb(self, msg):
        '''
        Once message is received on this topic, start process of displaying trajectory
        '''
        self.request_pub.publish(String("get_representation"))

    def _trajectory_cb(self, msg):

        # Clear out any visualizations currently present first
        self.hololens_pub.publish(String("CLEAR"))
        time.sleep(1.0)

        # Individually publish each keyframe to the HoloLens
        traj_object = json.loads(msg.data)
        traj_pusher = {}
        traj_pusher["trajectory"] = []
        for point_object in traj_object["point_array"]:
            # Update position and orientation
            pos = point_object["robot"]["position"]
            quat = point_object["robot"]["orientation"]
            updated_pose = self.transform_manager.world_to_hololens(Pose(
                Point(pos[0], pos[1], pos[2]), Quaternion(quat[0], quat[1], quat[2], quat[3])))

            # update JSON with transformed pose
            json_object = {}
            json_object["keyframe_id"] = point_object["keyframe_id"]
            json_object["applied_constraints"] = point_object["applied_constraints"]
            json_object["robot"] = {}
            json_object["robot"]["position"] = [updated_pose.pose.position.x,
                                                updated_pose.pose.position.y, updated_pose.pose.position.z]
            json_object["robot"]["orientation"] = [updated_pose.pose.orientation.x,
                                                   updated_pose.pose.orientation.y, updated_pose.pose.orientation.z, updated_pose.pose.orientation.w]

            traj_pusher["trajectory"].append(json_object)
        # transmit JSON for this transformed pose
        keyframe_msg = String()
        keyframe_msg.data = json.dumps(traj_pusher)
        self.hololens_pub.publish(keyframe_msg)

    def _constraint_cb(self, msg):
        constraint_msg = json.loads(msg.data)
        for constraint in constraint_msg["constraints"]:
            if(constraint["className"] == "HeightConstraintAbove" or constraint["className"] == "HeightConstraintBelow"):
                # Transform height constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(0, constraint["args"][0], 0), Quaternion(0, 0, 0, 1)))
                constraint["args"][0] = updated_pose.pose.position.z
            elif(constraint["className"] == "UprightConstraint"):
                # Transform upright constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(0, 0, 0), Quaternion(constraint["args"][0], constraint["args"][1],
                    constraint["args"][2], constraint["args"][3])))
                constraint["args"][0] = updated_pose.pose.orientation.x
                constraint["args"][1] = updated_pose.pose.orientation.y
                constraint["args"][2] = updated_pose.pose.orientation.z
                constraint["args"][3] = updated_pose.pose.orientation.w
            elif(constraint["className"] == "OverUnderConstraint"):
                # Transform over-under constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(constraint["args"][0], constraint["args"][1], constraint["args"][2]), 
                    Quaternion(0, 0, 0, 1)))
                constraint["args"][0] = updated_pose.pose.position.x
                constraint["args"][1] = updated_pose.pose.position.y
                #Subtract half a meter to make up for hololens visualization. TODO: fix this on HoloLens side
                constraint["args"][2] = updated_pose.pose.position.z - 0.5
            else:
                # Unsupported constraint type
                rospy.logwarn("Unsupported constraint type passed to transformer, being passed on as-is")

        transformed_constraint_msg = String()
        transformed_constraint_msg.data = json.dumps(constraint_msg)
        self.constraints_pub.publish(transformed_constraint_msg)


class ARPOLfDMiddleware():

    def __init__(self, ar_pos_transform=Vector3(0.975, -0.09, -0.27), ar_quat_transform=Quaternion(0.0, 0.0, 1.0, 0.0), pose_target_topic='arpo_lfd/pose_target', joint_configuration_topic="arpo_lfd/joint_configuration", playback_cmd_topic="arpo_lfd/trajectory_playback_cmd", joint_trajectory_topic="arpo_lfd/joint_trajectory"):
        
        
        # Create transform manager (position and axes hardcoded for now)
        self.transform_manager = ARVRFixedTransform("hololens", ar_pos_transform, ar_quat_transform, [[0, 0, 1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, -1]])

        
        # Callback stores
        self.current_optimized_pose = []
        self.current_trajectory = []
        self.current_pose_target = None
        
        # Initialize Publishers
        # Publish joint configurations for hololens as JSON encoded String
        self.joint_configuration_publisher = rospy.Publisher(joint_configuration_topic, String, queue_size=1)
        # Publish joint trajectory for hololens as JSON encoded String
        self.joint_trajectory_publisher = rospy.Publisher(joint_trajectory_topic, String, queue_size=1)
        # Pubblish EEPoseGoals to CollisionIK to get a solution
        self.ee_pose_goals_pub = rospy.Publisher('/collision_ik/ee_pose_goals', EEPoseGoals, queue_size=5)

        # Initialize Subscribers
        # Waits for a command to return the current trajectory for replay. 
        self.playback_sub = rospy.Subscriber(playback_cmd_topic, String, self._playback_cmd_cb)
        # Subscribes to Collision_IK_results
        self.collision_ik_sub =  rospy.Subscriber('/collision_ik/joint_angle_solutions', JointAngles, self._ja_solution_cb)
        self.target_pose_sub = rospy.Subscriber(pose_target_topic, PoseStamped, self._target_pose_cb)

        
        
    def _target_pose_cb(self, target_pose):
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(target_pose)
        ee_pose_goals.header.seq = seq
        seq += 1
        self.ee_pose_goals_pub.publish(ee_pose_goals)

    def _playback_cmd_cb(self, msg):
        if msg.data == "true":
            for configuration in self.current_trajectory:
                self.joint_configuration_publisher.publish(self._format_configuration_as_string(configuration))
        
    def _ja_solution_cb(self, data):
        ja_solution = []
        for a in data.angles.data:
            ja_solution.append(a)
        self.current_optimized_pose = ja_solution
        self.current_trajectory.append(ja_solution)
        # JSON formatted string to set to hololens.
        
        self.joint_configuration_publisher.publish(self._format_configuration_as_string(ja_solution))
    
    def _format_configuration_as_string(seld, configuration):
        data = {}
        data["joint_configuration"] = configuration
        return json.dumps(data)