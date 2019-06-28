"""
The analysis.py module contains a variety of methods and classes used to  analyze specific structures
and objects in the Cairo LfD ecosystem. This includes KeyframeGraphs, MotionPlans, and Constraints.
"""
import rospy
import numpy as np
import copy


class KeyframeGraphAnalyzer():
    """
    Supports the analysis of a KeyframeGraph.

    Attributes
    ----------
    graph : KeyframeGraph
       KeyframeGraph class object on which to perform analysis
    interface : robot_interface.moveit_interface.SawyerMoveitInterface
       Sawyer Robot interface.
    vectorizer : function
       Function that vectorizes Observation object.
    """
    def __init__(self, task_graph, sawyer_moveit_interface, observation_vectorizor):
        """
        Parameters
        ----------
        task_graph : KeyframeGraph
           KeyframeGraph class object on which to perform analysis
        sawyer_moveit_interface : robot_interface.moveit_interface.SawyerMoveitInterface
           Sawyer Robot interface.
        observation_vectorizor : function
           Function that vectorizes Observation object.
        """
        self.graph = task_graph
        self.interface = sawyer_moveit_interface
        self.vectorizor = observation_vectorizor

    def evaluate_keyframe_occlusion(self, keyframe_observations):
        """
        Evaluates a given list of keyframe observations for occlusion using the interface's check_point_validity()
        function.

        Parameters
        ----------
        keyframe_observations : list
           List of keyframe observations to evaluate. Expects Observation objects.

        Returns
        -------
        (free_observations, occluded_observations) : tuple
            A tuple containing a list of free observations as the first element and a list of occluded observations as the second element.
        """
        occluded_observations = []
        free_observations = []
        for observation in keyframe_observations:
            joints = observation.get_joint_angle()
            if joints is None:
                observation.data["robot"]["joints"] = self.interface.get_pose_IK_joints(observation.get_pose_list())
            if type(joints) is not list:
                joints = joints.tolist()
            if not self.interface.check_point_validity(self.interface.create_robot_state(joints)):
                occluded_observations.append(observation)
            else:
                free_observations.append(observation)
        return (free_observations, occluded_observations)

    def max_mean_ll(self, model, observation):
        """
        Given a set of observations, generate the mean log likelihood and max log likelihood of the set scored by a given model.

        Parameters
        ----------
        priod_model : object
            An modeling.models class object that has a score_samples() method.
        observations : list
           List of keyframe observations to evaluate their score.

        Returns
        -------
        (max_ll, mean_ll) : tuple
            Returns the maximum log likelihood of scored samples as well as the average of all the sample scores.
        """
        vectors = []
        for ob in observation:
            vector = self.vectorizor(ob)
            vectors.append(vector)
        vectors = np.array(vectors)
        curr_ll = model.score_samples(vectors)
        max_ll = np.amax(curr_ll)
        mean_ll = np.mean(curr_ll)
        return (max_ll, mean_ll)

    def cull_keyframes(self, threshold=-10000):
        """
        Culls consecutive keyframes sequentially until the one such keyframe's observations has an average LL is below
        a threshold as scored by the current keyframes model.

        Parameters
        ----------
        threshold : int
            The threshold average log likelihood.
        """
        prev = self.graph.get_keyframe_sequence()[0]
        curr = self.graph.successors(prev).next()
        while([x for x in self.graph.successors(curr)] != []):
            rospy.loginfo("Prev: {}; Curr: {}".format(prev, curr))
            max_ll, mean_ll = self.max_mean_ll(self.graph.nodes[prev]["model"], self.graph.nodes[curr]["observations"])
            if mean_ll > threshold and self.graph.nodes[curr]["keyframe_type"] != "constraint_transition":
                rospy.loginfo("Node {} average log-likelihood of {} is above threshold of {}".format(curr, mean_ll, threshold))
                succ = self.graph.successors(curr).next()
                self.graph.cull_node(curr)
                curr = succ
                continue
            prev = curr
            curr = self.graph.successors(curr).next()


class MotionPlanAnalyzer():
    """
    WIP: THIS CLASS IS NOT USABLE NOR COMPLETE
    Class with methods to support the analysis of a motion plan.

    Attributes
    ----------
    environment : Environment
        Environment object for the current LFD environment.
    """
    def __init__(self, environment):

        """
        Parameters
        ----------
        environment : Environment
           Environment object for the current LFD environment.
        """
        self.environment = environment

    def evaluate_plan(self, constraint_ids, plan_observations):
        """
        Evaluates plan observations to ensure that the constraints specified in the list of constraint ids
        is valid throughout the entire plan. If one observation in the plan violates the constraints,
        the plan is invalidated.

        Parameters
        ----------
        constraint_ids : list
            List of constraint id's to evaluate.

        plan_observations : list
            The observation to evaluate for the constraints.

        Returns
        -------
         : boolean
           Returns true if the given constraints are valid throughout the entirety of the plan, false otherwise.
        """
        for observation in plan_observations:
            evaluation = self.evaluate(constraint_ids, observation)
            # print constraint_ids, evaluation
            if constraint_ids != evaluation:
                return False
        return True

    def evaluate(self, constraint_ids, observation):
        """
        This function evaluates an observation for all the constraints in the list constraint_ids. It depends on
        being able to access the constraint objects from the self.environment object. Every constraint object
        should have an 'evaluate()'' function that takes in the environment and the observation.

        Parameters
        ----------
        constraint_ids : list
            List of constraint id's to evaluate.

        observation : Observation
            The observation to evaluate for the constraints.

        Returns
        -------
        valid_constraints : list
            Returns the list of valid constraints evaluated for the observation.
        """
        if constraint_ids != []:
            valid_constraints = []
            for constraint_id in constraint_ids:
                constraint = self.environment.get_constraint_by_id(constraint_id)
                result = constraint.evaluate(self.environment, observation)
                if result == 1:
                    valid_constraints.append(constraint_id)
            return valid_constraints
        else:
            return []


class ConstraintAnalyzer():
    """
    Constraint analysis class to evaluate observations for constraints.

    Attributes
    ----------
    environment : Environment
        Environment object for the current LFD environment.
    """
    def __init__(self, environment):
        """
        Parameters
        ----------
        environment : Environment
           Environment object for the current LFD environment.
        """
        self.environment = environment

    def applied_constraint_evaluator(self, observations):
        """
        This function evaluates observations for constraints that were triggered during the demonstration.
        It will label a demonstration's entire list of observations with the constraints that were triggered and whether or not they are still applicable.

        New constraints are those where the triggered constraints are different from the previously applied constraints:

            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered)-set(prev))

        The evaluated constraints are those that are still valid from the previous observation's applied constraints.

            evaluated = self.evaluate(constraint_ids=prev, observation=observation)

        The current observation's applied constraints are the union of the evaluated constraints and new constraints.

             applied = list(set(evaluated).union(set(new)))

        Parameters
        ----------
        observations : int
           List of observations to be evaluated for constraints.
        """
        rospy.loginfo("Analyzing observations for applied constraints...")
        prev = []
        for observation in observations:
            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered) - set(prev))
            evaluated = self.evaluate(constraint_ids=prev, observation=observation)
            applied = list(set(evaluated).union(set(new)))
            prev = applied
            observation.data["applied_constraints"] = applied

    def evaluate(self, constraint_ids, observation):
        """
        This function evaluates an observation for all the constraints in the list constraint_ids. It depends on
        being able to access the constraint objects from the self.environment object. Every constraint object
        should have an 'evaluate()'' function that takes in the environment and the observation.

        Parameters
        ----------
        constraint_ids : list
            List of constraint id's to evaluate.

        observation : Observation
            The observation to evaluate for the constraints.

        Returns
        -------
        valid_constraints : list
            Returns the list of valid constraints evaluated for the observation.
        """
        if constraint_ids != []:
            valid_constraints = []
            for constraint_id in constraint_ids:
                constraint = self.environment.get_constraint_by_id(constraint_id)
                result = constraint.evaluate(self.environment, observation)
                if result == 1:
                    valid_constraints.append(constraint_id)
            return valid_constraints
        else:
            return []