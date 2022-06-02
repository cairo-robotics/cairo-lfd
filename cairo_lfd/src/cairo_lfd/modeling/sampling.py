"""
The module sampling.py contains classes for sampling and ranking points from Keyframe models.
"""
import rospy
import numpy as np
from scipy.spatial.distance import euclidean

from cairo_lfd.modeling.analysis import check_constraint_validity, check_state_validity


class ConstrainedKeyframeModelSampler():
    """
    Sampling class that uses a keyframe's model to sample points for the Sawyer robot.

    Attributes
    ----------
    data_converter : func
        Function to convert a raw sample into an Observation object for use by the analyzer.
    """
    def __init__(self, sample_converter, robot_interface):
        """
        Parameters
        ----------
        data_converter : Object
            Object that manipulates a raw sample into an Observation object.
        """
        self.sample_converter = sample_converter
        self.robot_interface = robot_interface

    def sample(self, environment, model, primal_observation, constraints, n=100):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        primal_observation : Observation
            The most representative observation of the keyframe from which to derive all other metadata.
        constraints : list
            List of Constraint object on which to validate the sample for constraint validity.
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
            samples = model.generate_samples(1)
            if len(samples) > 0:
                sample = samples[0]
                observation = self.sample_converter.convert(sample, primal_observation, run_fk=True)
                occlusion_free = check_state_validity(observation, self.robot_interface)
                constraint_valid, matched_ids = check_constraint_validity(environment, constraints, observation)
                validated_set = validated_set.union(set(matched_ids))
                if constraint_valid and occlusion_free:
                    valid_samples.append(sample)
        return attempts, valid_samples, validated_set
    
    
class KeyframeModelSampler():
    """
    Sampling class that uses a keyframe's model to sample points. This is a very basic class for now that just directly samples
    from the mode, but could be expanded to check for constraint validity amongst other things.

    """
 
    def sample(self, model, n):
        """
        Parameters
        ----------
        model : object
            Model object used to generate raw samples.
        n : int
            Number of valid samples to generate.

        Returns
        -------
         : list
            List of valid raw samples generated from the model.
        """
        return model.generate_samples(n)


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


class ModelScoreRanking():

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


class ConfigurationSpaceRanking():

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

        distances = np.array([euclidean(prior_sample, sample) for sample in samples])
        order = np.argsort(distances)
        distances = distances[order]
        samples = np_array[order]
        rank_sorted_sampled = np.asarray(samples)
        return rank_sorted_sampled
