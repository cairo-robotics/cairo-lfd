"""python object wrappers for modeling"""
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture


class KMeansModel(object):

    """
    Wrapper class for Scikit Learn's KMeans clustering.
    """

    def __init__(self, observations, n_clusters=5):
        self.observations = observations
        self.n_clusters = n_clusters
        self.model = KMeans(n_clusters=self.n_clusters)

    def kmeans_fit(self):
        """
        Wrapper method for fit() method of kmeans model.
        """
        self.model.fit(self.observations)

    def get_cluster_samples(self):
        """
        Generates the raw samples associated with each cluster.

        Returns
        -------
        list
            List of numpy arrays of observations.
        """
        cluster_data = {}
        labels = self.model.labels_
        for i in range(0, self.n_clusters):
            cluster_data[i+1] = [self.observations[np.where(labels == i)]]
        return cluster_data


class GaussianMixtureModel(object):
    """
    Wrapper class for Scikit Learn's Gaussian Mixture Model.
    """
    def __init__(self, observations, n_components=5, means_init=None):
        self.observations = observations
        self.n_components = n_components
        self.model = GaussianMixture(n_components=self.n_components,
                                     covariance_type='full',
                                     means_init=means_init)

    def gmm_fit(self):
        """
        Wrapper method for fit() method of kmeans model.
        """
        self.model.fit(self.observations)

    def generate_samples(self, n_samples):
        """
        Generates the random samples according to the fitted distrubution.

        Returns
        -------
        list
            List of numpy arrays of randomly generated observations.

        """
        points, labels = self.model.sample(n_samples)
        return points
