import numpy as np
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from lfd_data.io import DataImporter
from lfd_environment.interfaces import Demonstration, Observation


def align_demonstrations(demo_1, demo_2):
    distance, path = fastdtw(demo_1, demo_2, dist=euclidean)
    print(distance)
    print(path)


if __name__ == "__main__":
    importer = DataImporter()
    trajectories = importer.load_json_files('./*.json')
    # x = np.array([[1,1], [2,2], [3,3], [4,4], [5,5]])
    # y = np.array([[2,2], [3,3], [4,4]])

    demonstrations = []
    for datum in trajectories["data"]:
        demo = Demonstration()
        observations = []
        for entry in datum:
            observation = Observation()
            observation.data = entry
            observations.append(observation)

    print demonstrations