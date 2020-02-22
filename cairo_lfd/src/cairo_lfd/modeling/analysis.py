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

    def max_mean_ll(self, model, observations):
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
        for ob in observations:
            vector = self.vectorizor(ob)
            vectors.append(vector)
        vectors = np.array(vectors)
        curr_ll = model.score_samples(vectors)
        max_ll = np.amax(curr_ll)
        mean_ll = np.mean(curr_ll)
        return (max_ll, mean_ll)

    def cull_keyframes(self, automate_threshold=False, culling_threshold=10):
        """
        Culls consecutive keyframes sequentially until the one such keyframe's has a KL divergence greater than the mean + 1 std KL divergence calculated from all consecutive keyframes.
        Parameters
        ----------
        culling_threshold : int
            The threshold value for culling if intending to pass a value manually.
        """
        if len(self.graph.get_keyframe_sequence()) > 0:
            if automate_threshold is True:
                average_KL_divergence, std_divergence = self._get_KL_divergence_stats()
                print(average_KL_divergence, std_divergence)
                prev = self.graph.get_keyframe_sequence()[0]
                curr = self.graph.successors(prev).next()
                while([x for x in self.graph.successors(curr)] != []):
                    est_divergence = self._KL_divergence_estimate(self.graph.nodes[prev]["model"], self.graph.nodes[curr]["model"])
                    if est_divergence < average_KL_divergence and self.graph.nodes[curr]["keyframe_type"] != "constraint_transition":
                        rospy.logwarn("KL estimate between nodes {} and {} is {} which below the mean divergence of {}".format(prev, curr, est_divergence, average_KL_divergence))
                        succ = self.graph.successors(curr).next()
                        self.graph.cull_node(curr)
                        curr = succ
                        continue
                    prev = curr
                    curr = self.graph.successors(curr).next()
            else:
                prev = self.graph.get_keyframe_sequence()[0]
                curr = self.graph.successors(prev).next()
                while([x for x in self.graph.successors(curr)] != []):
                    est_divergence = self._KL_divergence_estimate(self.graph.nodes[prev]["model"], self.graph.nodes[curr]["model"])
                    if est_divergence < culling_threshold and self.graph.nodes[curr]["keyframe_type"] != "constraint_transition":
                        rospy.logwarn("KL estimate between nodes {} and {} is {} which below set threshold of {}".format(prev, curr, est_divergence, culling_threshold))
                        succ = self.graph.successors(curr).next()
                        self.graph.cull_node(curr)
                        curr = succ
                        continue
                    prev = curr
                    curr = self.graph.successors(curr).next()

    def _get_KL_divergence_stats(self):
        print(self.graph)
        prev = self.graph.get_keyframe_sequence()[0]
        curr = self.graph.successors(prev).next()
        divergences = []
        while([x for x in self.graph.successors(curr)] != []):
            estimated_divergence = self._KL_divergence_estimate(self.graph.nodes[prev]["model"], self.graph.nodes[curr]["model"])
            divergences.append(estimated_divergence)
            prev = curr
            curr = self.graph.successors(curr).next()
        return np.average(divergences), np.std(divergences)

    def _KL_divergence_estimate(self, P, Q, n_samples=5 * 10**3):
        # uses a sampling based approach to estimate KL divergence
        # KL(p||q) = \int p(x) log(p(x) / q(x)) dx = E_p[ log(p(x) / q(x))
        X = P.generate_samples(n_samples)
        logP_X = P.score_samples(X)
        logQ_X = Q.score_samples(X)
        return np.average(logP_X) - np.average(logQ_X)


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
            prev_constraints = [c for c in self.environment.constraints if c.id in prev]
            valid, evaluated = self.evaluate(constraints=prev_constraints, observation=observation)
            applied = list(set(evaluated).union(set(new)))
            prev = applied
            observation.data["applied_constraints"] = applied

    def evaluate(self, constraints, observation):
        """
        This function evaluates an observation for a set of constraints. Every constraint object
        should have an 'evaluate()'' function that takes in the environment and the observation.

        Parameters
        ----------
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
        valid_ids = [constraint.id for constraint in constraints if constraint.evaluate(self.environment, observation)]
        valid_set = True if len(valid_ids) == len(constraints) else False
        return valid_set, valid_ids
