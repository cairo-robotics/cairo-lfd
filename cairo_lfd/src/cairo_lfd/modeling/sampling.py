"""
The module sampling.py contains classes for sampling and ranking points from Keyframe models.
"""
import numpy as np
import rospy


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
    def __init__(self, analyzer, data_converter):
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

    def generate_n_valid_samples(self, model, primal_observation, constraint_ids, n=100):
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
        attempts = 0
        while len(valid_samples) < n:
            attempts += 1
            if attempts >= n * 20:
                break
            samples = self.generate_raw_samples(model, 1)
            if len(samples) > 0:
                sample = samples[0]
                observation = self.converter.convert(sample, primal_observation, run_fk=True)
                matched_ids = self.analyzer.evaluate(constraint_ids, observation)
                if constraint_ids == matched_ids:
                    valid_samples.append(sample)
        return attempts, valid_samples, matched_ids

    def rank_samples(self, model, samples):
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
        scores = scores[order]
        rank_sorted_sampled = np.asarray(samples)
        return rank_sorted_sampled
