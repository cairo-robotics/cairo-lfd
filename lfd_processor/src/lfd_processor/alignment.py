from lfd_processor.interfaces import Demonstration, Observation
from lfd_processor.io import DataExporter
from lfd_processor.io import DataImporter
from dtw import fastdtw
from scipy.spatial.distance import euclidean
import itertools


def vectorize_demonstration(demonstration):
    obs1 = demonstration.vectorize_observations(["robot", "position"])
    obs2 = demonstration.vectorize_observations(["robot", "joints"])
    result = [i[0] + i[1] for i in zip(obs1, obs2)]
    return result


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
        return demonstrations

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


def get_constraint_ordering(demonstration):
    constraint_order = []
    curr = []
    for ob in demonstration.aligned_observations:
        if curr != ob.data["applied_constraints"]:
            constraint_order.append(ob.data["applied_constraints"])
            curr = ob.data["applied_constraints"]
    return constraint_order



if __name__ == "__main__":
    importer = DataImporter()
    trajectories = importer.load_json_files('./src/lfd/lfd_analyzer/src/lfd_analysis/*.json')

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
        print(get_constraint_ordering(demo))

    exp = DataExporter()
    for idx, demo in enumerate(aligned_demos):
        raw_data = [obs.data for obs in demo.aligned_observations]
        exp.export_to_json("./trajectory{}.json".format(idx), raw_data)

