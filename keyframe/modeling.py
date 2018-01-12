import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture




class KMeansModel:

    def __init__(self, observations, n_clusters=5):
        self.observations = observations
        self.n_clusters = n_clusters
        self.model = KMeans(n_clusters=self.n_clusters)

    def kmeans_fit(self):
        self.model.fit(self.observations)

    def get_cluster_samples(self):
        cluster_data = {}
        labels = self.model.labels_
        for i in range(0, self.n_clusters):
            cluster_data[i+1] = [self.observations[np.where(labels == i)]]
        return cluster_data


class GausssianMixtureModel:

    def __init__(self, observations, n_components=5, means_init=None):
        self.observations = observations
        self.n_components = n_components
        self.model = GaussianMixture(n_components=self.n_components, covariance_type='full', means_init=means_init)

    def gmm_fit(self):
        self.model.fit(self.observations)

    def generate_samples(self, n_samples):
        points, labels = self.model.sample(n_samples)
        return points


