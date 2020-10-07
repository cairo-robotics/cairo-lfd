""" Wrappers around various models"""
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture
from sklearn.neighbors.kde import KernelDensity


class KMeansModel(object):
    """
    Wrapper class for Scikit Learn's KMeans clustering.

    Attributes
    ----------
    n_clusters : int
        Number of clusters for KMeans
    model : KMeans
        Wrapped class model.
    """
    def __init__(self, n_clusters=5):
        """
        Attributes
        ----------
        n_clusters : int
            Number of clusters for KMeans
        """
        self.n_clusters = n_clusters
        self.model = KMeans(n_clusters=self.n_clusters)

    def fit(self, train_X):
        """
        Wrapper method for fit() method of kmeans model.
        """
        self.model.fit(train_X)

    def get_clusters(self, train_X):
        """
        Generates the raw samples associated with each cluster.

        Parameters
        ----------
        train_X : {array-like, sparse matrix}, shape = [n_samples, n_features]

        Returns
        -------
        cluster_data : dict
            Dictionary of clusters.
        """
        cluster_data = {}
        labels = self.model.labels_
        for i in range(0, self.n_clusters):
            cluster_data[i+1] = [train_X[np.where(labels == i)]]
        return cluster_data

    def score_samples(self, X):
        """
        Predicts which cluster each of the samples in X belongs.

        Parameters
        ----------
        X : {array-like, sparse matrix}, shape = [n_samples, n_features]
        """
        return self.model.predict(X)


class GaussianMixtureModel(object):
    """
    Wrapper class for Scikit Learn's Gaussian Mixture Model.

    Attributes
    ----------
    n_components : int
        Number of clusters for KMeans
    model : GaussianMixture
        Wrapped class model.
    """
    def __init__(self, n_components=5, means_init=None):
        """
        Parameters
        ----------
        n_components : int
            Number of GMM components.
        means_init : list
            List of length n_components of numerical values for initial means of GMM.
        """
        self.n_components = n_components
        self.model = GaussianMixture(n_components=self.n_components,
                                     covariance_type='full',
                                     means_init=means_init)

    def fit(self, train_X):
        """
        Wrapper method for fit() method of GMM model.

        Parameters
        ----------
        train_X : {array-like, sparse matrix}, shape = [n_samples, n_features]
        """
        self.model.fit(train_X)

    def generate_samples(self, n_samples):
        """
        Generates the random samples according to the fitted distribution.

        Returns
        -------
        list
            List of numpy arrays of randomly generated observations.
        """
        points, labels = self.model.sample(n_samples)
        return points

    def score_samples(self, X):
        """
        Predicts the log likelihood score of the samples in X.

        Parameters
        ----------
        X : {array-like, sparse matrix}, shape = [n_samples, n_features]
        """
        return self.model.predict_proba(X)


class KDEModel(object):
    """
    Wrapper class for Scikit Learn's Kernel Density Estimation model.

    Attributes
    ----------
    model : KernelDensity
        Wrapped class model.
    """
    def __init__(self, kernel='gaussian', bandwidth=.001):
        self.model = KernelDensity(kernel='gaussian', bandwidth=bandwidth)

    def fit(self, train_X):
        """
        Wrapper method for fit() method of Kernel Density model.

        Parameters
        ----------
        train_X : {array-like, sparse matrix}, shape = [n_samples, n_features]
        """
        self.model.fit(train_X)

    def generate_samples(self, n_samples):
        """
        Generates the random samples according to the fitted distribution.

        Returns
        -------
        list
            List of numpy arrays of randomly generated observations.

        """
        points = self.model.sample(n_samples)
        return points

    def score_samples(self, X):
        """
        Predicts the log likelihood score of the samples in X.

        Parameters
        ----------
        X : {array-like, sparse matrix}, shape = [n_samples, n_features]
        """
        return self.model.score_samples(X)
