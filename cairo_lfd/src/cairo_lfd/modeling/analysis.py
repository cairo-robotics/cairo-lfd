"""
The analysis.py module contains a variety of methods and classes used to  analyze specific structures
and objects in the Cairo LfD ecosystem. This includes KeyframeGraphs, MotionPlans, and Constraints.
"""
import rospy
import copy

from cairo_lfd.modeling.stats import kullbach_leibler_divergence, model_divergence_stats


def evaluate_applied_constraints(environment, observations):
    """
    This function evaluates observations for constraints that were triggered during the demonstration.
    It will label a demonstration's entire list of observations with the constraints that were triggered and whether or not they are still applicable.

    New constraints are those where the triggered constraints are different from the previously applied constraints:

        triggered = observation.get_triggered_constraint_data()
        new = list(set(triggered)-set(prev))

    The evaluated constraints are those that are still valid from the previous observation's applied constraints.

        evaluated = evaluate(constraint_ids=prev, observation=observation)

    The current observation's applied constraints are the union of the evaluated constraints and new constraints.

         applied = list(set(evaluated).union(set(new)))

    Parameters
    ----------
    environment : Environment
         Environment object for the current LFD environment.
    observations : int
       List of observations to be evaluated for constraints.
    """
    rospy.loginfo("Analyzing observations for applied constraints...")
    prev = []
    for observation in observations:
        triggered = observation.get_triggered_constraint_data()
        new = list(set(triggered) - set(prev))
        prev_constraints = [c for c in environment.constraints if c.id in prev]
        _, valid_ids = check_constraint_validity(environment, constraints=prev_constraints, observation=observation)
        applied = list(set(valid_ids).union(set(new)))
        prev = applied
        observation.data["applied_constraints"] = applied


def check_constraint_validity(environment, constraints, observation):
    """
    This function evaluates an observation for a set of constraints. Every constraint object
    should have an 'evaluate()'' function that takes in the environment and the observation.

    Parameters
    ----------
    environment : Environment
         Environment object for the current LFD environment.
    constraints : list
        List of constraint objects to evaluate.
    observation : Observation
        The observation to evaluate for the constraints.

    Returns
    -------
    valid_ids : list
        List of valid constraints ids evaluated for the observation.
    valid_set : bool
        Indicator of whether or not all constraints are valid.
    """
    valid_ids = [constraint.id for constraint in constraints if constraint.evaluate(environment, observation)]
    valid_set = True if len(valid_ids) == len(constraints) else False
    return valid_set, valid_ids


def check_state_validity(observation, robot_interface):
    """
    Evaluates an observations for occlusion using the a robot interface's check_point_validity()
    function.

    Parameters
    ----------
    observation : Observation
       Observations to evaluate.
    robot_interface : RobotInterface from cairo_robot_interface package

    Returns
    -------
    : bool
        Whether or not the observation constitutes a valid robot state.
    """

    joints = observation.get_joint_angle()
    if joints is None:
        observation.data["robot"]["joints"] = robot_interface.get_pose_IK_joints(observation.get_pose_list())
    if type(joints) is not list:
        joints = joints.tolist()
    if not robot_interface.check_point_validity(robot_interface.create_robot_state(joints)):
        return False
    else:
        return True


def get_culling_candidates(graph, automate_threshold=False, culling_threshold=10):
    """

    Generates candidate node ID's from a KeyframeGraph representing keyframes that are consecutively too "close" according to the automated or passed in culling threshold according the Kullbach Leibler Divergence between keyframe models.

    Parameters
    ----------
    graph : KeyframeGraph
        Graph from which to generate candidate keyframe
    automate_threshold : bool
        Indicates whether or not to use automated thresholding for culling candidacy
    culling_threshold : int
        The threshold value for culling if intending to pass a value manually.
    """
    candidate_ids = []

    if len(graph.get_keyframe_sequence()) > 0:
        if automate_threshold is True:
            average_KL_divergence, std_divergence = model_divergence_stats(graph)
            prev = graph.get_keyframe_sequence()[0]
            curr = graph.successors(prev).next()
            while([x for x in graph.successors(curr)] != []):
                est_divergence = kullbach_leibler_divergence(graph.nodes[prev]["model"], graph.nodes[curr]["model"])
                if est_divergence < average_KL_divergence and graph.nodes[curr]["keyframe_type"] != "constraint_transition":
                    rospy.logwarn("KL estimate between nodes {} and {} is {} which below the mean divergence of {}".format(prev, curr, est_divergence, average_KL_divergence))
                    succ = graph.successors(curr).next()
                    candidate_ids.append(curr)
                    curr = succ
                    continue
                prev = curr
                curr = graph.successors(curr).next()
        else:
            prev = graph.get_keyframe_sequence()[0]
            curr = graph.successors(prev).next()
            while([x for x in graph.successors(curr)] != []):
                est_divergence = kullbach_leibler_divergence(graph.nodes[prev]["model"], graph.nodes[curr]["model"])
                if est_divergence < culling_threshold and graph.nodes[curr]["keyframe_type"] != "constraint_transition":
                    rospy.logwarn("KL estimate between nodes {} and {} is {} which below set threshold of {}".format(prev, curr, est_divergence, culling_threshold))
                    succ = graph.successors(curr).next()
                    candidate_ids.append(curr)
                    curr = succ
                    continue
                prev = curr
                curr = graph.successors(curr).next()
    return candidate_ids