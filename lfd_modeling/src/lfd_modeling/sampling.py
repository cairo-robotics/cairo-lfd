import numpy as np
import rospy

from lfd_processor.environment import Observation


class KeyframeSampler():

    def __init__(self, analyzer, data_converter):
        self.analyzer = analyzer
        self.converter = data_converter

    def generate_raw_samples(self, keyframe_node, num_of_samples):
        """
        wrapper for sampling points
        return obsv objects
        """
        raw_samples = keyframe_node[0]["model"].sample(num_of_samples)
        return raw_samples

    def generate_n_valid_samples(self, keyframe_node, constraint_ids, n=100):
        """
        returns n valid samples based on the constraints
        """

        valid_samples = []
        attempts = 0
        while len(valid_samples) < n:
            attempts += 1
            if attempts >= n * 20:
                break
            samples = self.generate_raw_samples(keyframe_node, 1)
            if len(samples) > 0:
                sample = samples[0]
                matched_ids = self.analyzer.evaluate(constraint_ids, self.converter.convert(sample))
                if constraint_ids == matched_ids:
                    valid_samples.append(sample)

        rospy.loginfo("%s valid of %s attempts", len(valid_samples), attempts)
        if len(valid_samples) < n:
            rospy.logwarn("only %s of %s waypoints provided", len(valid_samples), n)
        if len(valid_samples) == 0:
            rospy.loginfo("Node {} has no valid sample observations".format(keyframe_node[0]))
        return valid_samples

    def rank_samples(self, keyframe_node, samples):
        """
        re arrange all sampled points based on Prob Density
        """

        model = keyframe_node["model"]
        np_array = []
        for sample in samples:
            np_array.append(sample)

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        sorted_sampled = np.asarray(samples)
        return sorted_sampled

    def rank_waypoint_free_samples(self, node_id):
        """
        re arrange all sampled points based on Prob Density
        """
        # TODO add desnity values to samples?

        model = self.nodes[node_id]["model"]
        samples = self.nodes[node_id]['free_samples']

        np_array = []
        for sample in samples:
            np_array.append(np.array(self.vectorizor(sample)))

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        samples = np.asarray(samples)
        self.nodes[node_id]['free_samples'] = samples
        rospy.loginfo("keyframe %s samples have been reorderd", node_id)
        return 0