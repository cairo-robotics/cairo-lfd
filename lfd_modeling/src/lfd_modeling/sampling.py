import numpy as np
import rospy


class KeyframeSampler():

    def __init__(self, analyzer, data_converter):
        self.analyzer = analyzer
        self.converter = data_converter

    def generate_raw_samples(self, model, num_of_samples):
        """
        wrapper for sampling points
        return obsv objects
        """
        raw_samples = model.generate_samples(num_of_samples)
        return raw_samples

    def generate_n_valid_samples(self, model, constraint_ids, n=100):
        """
        returns n valid samples based on the constraints
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
                matched_ids = self.analyzer.evaluate(constraint_ids, self.converter.convert(sample))
                # print(constraint_ids, matched_ids)
                if constraint_ids == matched_ids:
                    valid_samples.append(sample)
        return attempts, valid_samples

    def rank_samples(self, model, samples):
        """
        re arrange all sampled points based on Prob Density
        """
        if len(samples) == 0:
            rospy.logwarn("No samples to rank.")
            return []
        array = []
        for sample in samples:
            array.append(sample)
        np_array = np.array(array)

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        sorted_sampled = np.asarray(samples)
        return sorted_sampled
