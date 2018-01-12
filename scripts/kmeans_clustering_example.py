from keyframe.data_processing import DataProcessor, DataImporter
from keyframe.visualization import KMeansModelViewer
from keyframe.modeling import KMeansModel

if __name__  == "__main__":

    """
    An example of generating and visualizing KMeans on the raw trajectories from the toy data. 
    """

    importer = DataImporter()
    processor = DataProcessor()

    trajectories_dict = importer.import_csv_to_dict('../toy_data/raw_trajectories/*.csv')
    observations = []
    for t in trajectories_dict["trajectories"]:
        for observation in t["observations"]:
            observations.append(processor.convert_observation_dict_to_list(observation, key_order=["PoseX", "PoseY", "PoseZ"]))
    np_observation = processor.to_np_array(observations)

    kmm = KMeansModel(np_observation, n_clusters=5)
    kmm.kmeans_fit()

    viewer = KMeansModelViewer(kmm, np_observation)
    viewer.view_3D_clusters(0, 1, 2)
