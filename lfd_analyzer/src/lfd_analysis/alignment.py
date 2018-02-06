from scipy.spatial.distance import euclidean
from lfd_data.io import DataImporter
from lfd_environment.interfaces import Demonstration, Observation
from dtw import fastdtw


def vectorize_demonstration(self, demonstrations):
        vectorized_demonstrations = []
        for demo in demonstrations:
            vectorized_demonstrations.append(demo.vectorize_observations(["robot", "position"]))
        return vectorized_demonstrations


class DemonstrationAligner(object):

    def __init__(self, demonstrations, vectorize_func):
        self.demonstrations = demonstrations
        self.vectorize_func = vectorize_func

    def align(self):
        self.demonstrations.sort(key = lambda d: len(d.observations.data))
        reference_demo = self.demonstrations[-1]
        aligned_demos = []
        for idx, curr_demo in enumerate(self.demonstrations):
            if idx == 0:
                new_demos = self.align_two_demonstrations(curr_demo, reference_demo)
                aligned_demos = aligned_demos + new_demos
                reference_demo = new_demos[1]
            else:
                new_demos = self.align_two_demonstrations(curr_demo, reference_demo)[0]
                aligned_demos.append(self.align_two_demonstrations(curr_demo, reference_demo)[0])
                reference_demo = new_demos[1]
        return aligned_demos

    def align_two_demonstrations(self, demo1, demo2):
        demos = [demo1, demo2]
        demos.sort(key = lambda d: len(d.observations.data))

        demo_vectors = [self.vectorize_func(demo) for demo in demos]

        dist, cost, acc, path = fastdtw(demo_vectors[0], demo_vectors[1], dist=euclidean)
        idx_pairs = zip(path[0].tolist(), path[1].tolist())

        aligned_vector_1 = []
        aligned_vector_2 = []
        for idx, pair in enumerate(pairs):
            # build new osbervation trajectory
            demo1 = demos[0].get_observation_by_index(pair[0])
            demo2 = demos[1].get_observation_by_index(pair[1])
            constraint_union = list(set(demo1.data["applied_constraints"] + demo2.data["applied_constraints"]))
            demo1.data["applied_constraints"] = constraint_union
            demo2.data["applied_constraints"] = constraint_union
            new_x_trajectory.append(demo1)
            new_y_trajectory.append(demo2)

        return [Demonstration(new_x_trajectory), Demonstration(new_y_trajectory)]


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


    self.


    # Vectorize data
    position_vectors = []   
    for demo in demonstrations:
        position_vectors.append(demo.vectorize_observations(["robot", "position"]))

    position_vectors.sort(key = lambda s: len(s))

    for v in position_vectors:
        print(len(v))

    dist, cost, acc, path = fastdtw(position_vectors[0], position_vectors[1], dist=euclidean)
    
    pairs = zip(path[0].tolist(), path[1].tolist())

    new_x_trajectory = []
    new_y_trajectory = []
    for idx, pair in enumerate(pairs):
        # build new osbervation trajectory
        demo1 = demonstrations[0].get_observation_by_index(pair[0])
        demo2 = demonstrations[1].get_observation_by_index(pair[1])
        constraint_union = list(set(demo1.data["applied_constraints"] + demo2.data["applied_constraints"]))
        demo1.data["applied_constraints"] = constraint_union
        demo2.data["applied_constraints"] = constraint_union
        new_x_trajectory.append(demo1)
        new_y_trajectory.append(demo2)

    print len(new_x_trajectory)
    print len(new_y_trajectory)
    new_x_demo = Demonstration(new_x_trajectory)
    new_y_demo = Demonstration(new_y_trajectory)




