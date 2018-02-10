from lfd_processor.environment import Demonstration, Observation
from lfd_processor.data_io import DataExporter
from lfd_processor.data_io import DataImporter
from dtw import fastdtw
from scipy.spatial.distance import euclidean


def vectorize_demonstration(demonstration):
    vectors = []
    for observation in demonstration.observations:
        position_data = observation.data["robot"]["position"]
        joint_data = observation.data["robot"]["joints"]
        vector = position_data + joint_data
        vectors.append(vector)
    return vectors


class DemonstrationAligner(object):

    def __init__(self, demonstrations, vectorize_func):
        self.demonstrations = demonstrations
        self.vectorize_func = vectorize_func

    def align(self):
        self.demonstrations.sort(key = lambda d: len(d.observations))
        reference_demo = self.demonstrations[1]
        aligned_demos = []
        for idx, curr_demo in enumerate(self.demonstrations):
            # first loop collects applied constraints into longest demonstration as master reference
            alignments = self.get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        for idx, curr_demo in enumerate(self.demonstrations):
            alignments = self.get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        for idx, curr_demo in enumerate(self.demonstrations):
            # by third loop, constraints have converged to an equivalent mapping.
            # I do not like or know exactly why but intuitively it makes some sense as iteratively running DTW will
            # converge on some global alignment if a reference vector is always used.
            alignments = self.get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        return self.demonstrations

    def get_alignment(self, current_demo, reference_demo):
        demos = [current_demo, reference_demo]
        demo_vectors = [self.vectorize_func(demo) for demo in demos]
        dist, cost, acc, path = fastdtw(demo_vectors[0], demo_vectors[1], dist=euclidean)
        idx_pairs = zip(path[0].tolist(), path[1].tolist())

        current_aligned_observations = []
        reference_aligned_observations = []
        for idx, pair in enumerate(idx_pairs):
            # build new osbervation trajectory
            current_ob = demos[0].get_observation_by_index(pair[0])
            reference_ob = demos[1].get_observation_by_index(pair[1])
            constraint_union = list(set(current_ob.data["applied_constraints"] + reference_ob.data["applied_constraints"]))
            current_ob.data["applied_constraints"] = constraint_union
            reference_ob.data["applied_constraints"] = constraint_union
            current_aligned_observations.append(current_ob)
            reference_aligned_observations.append(reference_ob)
        return {
            "current": current_aligned_observations,
            "reference": reference_aligned_observations
        }


if __name__ == "__main__":
    importer = DataImporter()
    trajectories = importer.load_json_files('./src/lfd/lfd_processor/src/lfd_processor/*.json')

    # Convert trajectory data into Demonstrations and Observations
    demonstrations = []
    for datum in trajectories["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    aligner = DemonstrationAligner(demonstrations, vectorize_demonstration)
    aligned_demos = aligner.align()

    print "Demonstration Constraint Transitions"
    for demo in aligned_demos:
        print demo.get_applied_constraint_order()

    exp = DataExporter()
    for idx, demo in enumerate(aligned_demos):
        raw_data = [obs.data for obs in demo.aligned_observations]
        exp.export_to_json("./trajectory{}.json".format(idx), raw_data)

