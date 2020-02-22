"""
The module sampling.py contains classes for sampling and ranking points from Keyframe models.
"""
import numpy as np
import rospy

from cairo_lfd.data.processing import EuclideanDistanceMixin


class KeyframeSampler():
    """
    Sampling class that uses a keyframe's model to sample points.

    Attributes
    ----------
    analyzer : object
        Analysis object that evaluates sampled points for their validity (constraint satisfcation).
    data_converter : func
        Function to convert a raw sample into an Observation object for use by the analyzer.
    """
    def __init__(self, analyzer, data_converter, robot_interface):
        """
        Parameters
        ----------
        analyzer : ConstraintAnalyzer
            Evaluates sampled points for their validity (constraint satisfcation).
        data_converter : func
            Function to convert a raw sample into an Observation object for use by the analyzer.
        """
        self.analyzer = analyzer
        self.converter = data_converter
        self.interface = robot_interface

    def generate_raw_samples(self, model, num_of_samples):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        num_of_samples : int
            Number of samples to generate.

        Returns
        -------
        raw_samples : list
            List of raw samples generated from the model.
        """
        raw_samples = model.generate_samples(num_of_samples)
        return raw_samples

    def generate_n_valid_samples(self, model, primal_observation, constraints, n=100):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        constraint_ids : list
            List of constraint IDs required for validity.
        n : int
            Number of valid samples to generate.

        Returns
        -------
        raw_samples : list
            List of valid raw samples generated from the model.
        """
        valid_samples = []
        validated_set = set()
        attempts = 0
        while len(valid_samples) < n:
            attempts += 1
            if attempts >= n * 10:
                break
            samples = self.generate_raw_samples(model, 1)
            if len(samples) > 0:
                sample = samples[0]
                observation = self.converter.convert(sample, primal_observation, run_fk=True)
                occlusion_free = self.evaluate_keyframe_occlusion(observation)
                constraint_valid, matched_ids = self.analyzer.evaluate(constraints, observation)
                validated_set = validated_set.union(set(matched_ids))
                if constraint_valid and occlusion_free:
                    valid_samples.append(sample)
        return attempts, valid_samples, validated_set

    def evaluate_keyframe_occlusion(self, observation):
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

        joints = observation.get_joint_angle()
        if joints is None:
            observation.data["robot"]["joints"] = self.interface.get_pose_IK_joints(observation.get_pose_list())
        if type(joints) is not list:
            joints = joints.tolist()
        if not self.interface.check_point_validity(self.interface.create_robot_state(joints)):
            return False
        else:
            return True


class AutoconstraintSampler():

    def __init__(self, autoconstraint_dict):
        self.autoconstraint_dict = autoconstraint_dict
        self.current_set = set()

    def validate(self, validated_set):
        if self.current_set == set():
            return False
        missing_constraints_name_id = self.current_set.difference(validated_set)
        if missing_constraints_name_id == set():
            return True
        else:
            return False

    def sample(self, validated_set):
        if self.current_set == set():
            for name, autoconstraint in self.autoconstraint_dict.items():
                self.current_set.add((name, 0))
        elif self.current_set.difference(validated_set) == set():
            self.current_set.clear()
            for name, idx in list(validated_set):
                if idx + 1 < len(self.autoconstraint_dict[name].constraints):
                    self.current_set.add((name, idx + 1))
                else:
                    self.current_set.add((name, idx))
        else:
            missing_constraints_name_id = self.current_set.difference(validated_set)
            self.current_set = validated_set
            for name, idx in list(missing_constraints_name_id):
                if idx + 1 < len(self.autoconstraint_dict[name].constraints):
                    self.current_set.add((name, idx + 1))
        return [self.autoconstraint_dict[name_id[0]].constraints[name_id[1]] for name_id in self.current_set]


class ModelScoreSampleRanker():

    def rank(self, model, samples):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        samples : list
            List of samples to rank according to their score as measured by the model.

        Returns
        -------
        rank_sorted_sampled : list
            List of rank (according to model scoring function) sorted samples (descending order).
        """
        if len(samples) == 0:
            rospy.logwarn("No samples to rank.")
            return []
        array = []
        for sample in samples:
            array.append(sample)
        np_array = np.array(array)

        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        samples = np_array[order]
        rank_sorted_sampled = np.asarray(samples)
        return rank_sorted_sampled


class ConfigurationSpaceSampleRanker(EuclideanDistanceMixin):

    def rank(self, model, samples, prior_sample):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        samples : list
            List of samples to rank according to their score as measured by the model.

        Returns
        -------
        rank_sorted_sampled : list
            List of rank (according to model scoring function) sorted samples (descending order).
        """
        if len(samples) == 0:
            rospy.logwarn("No samples to rank.")
            return []
        array = []
        for sample in samples:
            array.append(sample)
        np_array = np.array(array)

        distances = np.array([self._euclidean(prior_sample, sample) for sample in samples])
        order = np.argsort(distances)
        distances = distances[order]
        samples = np_array[order]
        rank_sorted_sampled = np.asarray(samples)
        return rank_sorted_sampled
