from keyframe.data_processing import DataProcessor, DataImporter
from keyframe.modeling import KMeansModel
import json

if __name__  == "__main__":

    importer = DataImporter()
    processor = DataProcessor()

    trajectories_dict = importer.import_csv_to_dict('../toy_data/raw_trajectories/*.csv')
    observations = []
    for t in trajectories_dict["trajectories"]:
        for observation in t["observations"]:
            observations.append(processor.convert_observation_dict_to_list(observation, key_order=["PoseX", "PoseY", "PoseZ"]))
    np_observation = processor.convert_to_numpy_array(observations)

    km_clusterer = KMeansModel(np_observation, n_clusters=5)
    km_clusterer.kmeans_fit()

    # Obtain the cluster order by iterating over the data.
    cluster_order = []
    for t in trajectories_dict["trajectories"]:
        for observation in t["observations"]:
            sample = processor.convert_to_numpy_array([processor.convert_observation_dict_to_list(observation, key_order=["PoseX", "PoseY", "PoseZ"])])
            cluster = int(km_clusterer.kmeans.predict(sample)[0])
            if cluster not in cluster_order:
                cluster_order.append(cluster)

    #
    cluster_data = {}
    for cluster in cluster_order:
        cluster_data[cluster] = []
    for t in trajectories_dict["trajectories"]:
        for observation in t["observations"]:
            sample = processor.convert_to_numpy_array(
                [processor.convert_observation_dict_to_list(observation, key_order=["PoseX", "PoseY", "PoseZ"])])
            cluster = int(km_clusterer.kmeans.predict(sample)[0])
            cluster_data[cluster].append(observation)


    keyframes = {}
    counter = 1
    for n, cluster in enumerate(cluster_order):
        curr_observations = cluster_data[cluster]
        keyframe_data = curr_observations
        keyframes[str(counter)] = {"data": keyframe_data, "type": "keyframe"}

        if(n+1 < len(cluster_order)):
            next_cluster = cluster_order[n+1]
            next_observations = cluster_data[next_cluster]
            transition_data = curr_observations[-12:] + next_observations[:12]
            keyframes[str(counter+1)] = {"data": transition_data, "type": "transition"}
            counter += 1
        counter += 1

    with open("../toy_data/keyframe_data/keyframes.json", 'w') as f:
        json.dump(keyframes, f, indent=4, sort_keys=True)
