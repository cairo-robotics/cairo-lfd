import json
import time

import rospy

from std_msgs.msg import String, Float64MultiArray, Bool
from geometry_msgs.msg import Pose, Point, Quaternion


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

    def __init__(self, transform_manager, command_topic, request_topic, traj_representation_topic, hololens_pub_topic, constraint_edits_in, constraint_edits_out):
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
        self.transform_manager = transform_manager

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

    def __init__(self, transform_manger, joint_configuration_topic="arpo_lfd/joint_configuration", playback_cmd_topic="arpo_lfd/trajectory_playback_cmd", joint_trajectory_topic="arpo_lfd/joint_trajectory"):
        
        
        self.transform_manager = transform_manger

        # Callback stores
        self.current_trajectory = []
        self.current_pose_target = None
        
        # Initialize Publishers
        # Publish joint configurations for hololens as JSON encoded String
        self.joint_configuration_publisher = rospy.Publisher(joint_configuration_topic, String, queue_size=1)
        # Publish joint trajectory for hololens as JSON encoded String
        self.joint_trajectory_publisher = rospy.Publisher(joint_trajectory_topic, String, queue_size=1)

        # Initialize Subscribers
        # Waits for a command to return the current trajectory for replay. 
        self.playback_sub = rospy.Subscriber(playback_cmd_topic, String, self._playback_cmd_cb)
        # Subscribes to IK results coming from the recording process.
        self.joint_angle_solution_sub = rospy.Subscriber('arpo_lfd/joint_angles', Float64MultiArray, self._ja_solution_cb)
        # If msg.data is True, the current trajectory is cleared.
        self.clear_trajectory_cmd_sub = rospy.Subscriber('apro_lfd/clear_traj_cmd', Bool, self._clear_traj)
        
        
    def _playback_cmd_cb(self, msg):
        if msg.data == "true":
            for configuration in self.current_trajectory:
                self.joint_configuration_publisher.publish(self._format_configuration_as_string(configuration))
                time.sleep(.1)
        
    def _ja_solution_cb(self, msg):
        ja_solution = []
        for a in msg.data:
            ja_solution.append(a)
        self.current_trajectory.append(ja_solution)
        # JSON formatted string to set to hololens.
        self.joint_configuration_publisher.publish(self._format_configuration_as_string(ja_solution))
    
    def _clear_traj(self, msg):
        if msg.data is True:
            self.current_trajectory = []
    
    def _format_configuration_as_string(seld, configuration):
        data = {}
        data["joint_configuration"] = configuration
        return json.dumps(data)