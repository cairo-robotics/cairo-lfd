"""
The visualization.py module constains classes for viewing sample poitns and models used in Cairo LfD.
"""
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib.colors import colorConverter
from mpl_toolkits.mplot3d import Axes3D
import randomcolor
import numpy as np


class Scatter3D:
    """
    Class for viewing samples points via Matplotlib's pyplot.
    """
    def plot(self, sample_points, x_index, y_index, z_index):
        """
        Generates 3D graph according of the passed in sample points.

        Parameters
        ----------
        sample_points : list
            List of numpy arrays (usually observations)
        x_index : int
            Index of sample point to represent the x-axis value.
        y_index : int
            Index of sample point to represent the y-axis value.
        z_index : int
            Index of sample point to represent the z-axis value.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.autoscale_view()
        ax.scatter(sample_points[:, x_index], sample_points[:, y_index], sample_points[:, z_index])
        plt.show()


class GaussianMixture3D():
    """
    Class for viewing sample points, their assignment to components of a mixture model, and the 
    corresponding Gaussian ellipsoids as 3D wire frames representing one standard deviations.

    """
    def plot(X, Y_, means, covariances):
        """
        Generates 3D graph of scatter plotted sample points, colored by label, overlayed with
        the Gaussian Mixture component ellipsoids.

        This should support any Gaussian Mixture model implemented in the guise of SciKit learn
        BayesianGaussianMixxture or GaussianMixture where X is similar in shape to training data, 
        Y_ is results provided by predict() function, and the means and covariances are attributes
        of the class model.

        Parameters
        ----------
        X : ndarray
            Numpy array of shape (n_samples, 3) where number of dimensions is restricted to 3. May
            require projecting higher level state space to 3D space.
        Y_ : array-like
            Array of the component labels for every sample.
        means : array-like
            Means of Gaussian components
        z_index : int
            Covariances of Gaussian components
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i, (mean, covar, color) in enumerate(zip(
                means, covariances, color_iter)):

            # as the DP will not use every component it has access to
            # unless it needs it, we shouldn't plot the redundant
            # components.
            if not np.any(Y_ == i):
                continue
            ax.scatter(X[Y_ == i, 0], X[Y_ == i, 1], X[Y_ == i, 2], s=.8, color=color)

            # find the rotation matrix and radii of the axes
            U, s, rotation = linalg.svd(covar)
            radii = np.sqrt(s)

            # now carry on with EOL's answer
            u = np.linspace(0.0, 2.0 * np.pi, 100)
            v = np.linspace(0.0, np.pi, 100)
            x = radii[0] * np.outer(np.cos(u), np.sin(v))
            y = radii[1] * np.outer(np.sin(u), np.sin(v))
            z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
            for i in range(len(x)):
                for j in range(len(x)):
                    [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + mean

            # plot
            ax.plot_wireframe(x, y, z, rstride=4, cstride=4, color=color, alpha=0.2)
        plt.show()


class KMeans2D:
    """
    Class for viewing observation vector representation with KMeansModel.

    Attributes
    ----------
    kmm : KMeansModel
        Model to visualize.
    observation_vectors : list
        List of vectorized Observations.
    """
    def __init__(self, kmm, observation_vectors):
        """
        Parameters
        ----------
        kmm : KMeansModel
            Model to visualize.
        observation_vectors : list
            List of vectorized Observations.
        """
        self.kmm = kmm
        self.observation_vectors = observation_vectors

    def plot(self, x_index, y_index, z_index):
        """
        Generates 3D graph of the clusters of KMeans model

        Parameters
        ----------
        x_index : int
            Index of sample point to represent the x-axis value.
        y_index : int
            Index of sample point to represent the y-axis value.
        z_index : int
            Index of sample point to represent the z-axis value.
        """
        labels = self.kmm.model.labels_
        centroids = self.kmm.model.cluster_centers_

        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.kmm.n_clusters))]

        fig = plt.figure(figsize=(10, 10))
        ax = Axes3D(fig)
        ax.autoscale_view()

        X = self.observation_vectors

        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            ax.scatter(data[:, x_index], data[:, y_index], data[:, z_index], color=color)
        ax.scatter(centroids[:, x_index], centroids[:, y_index], centroids[:, z_index], marker="x", s=150, linewidths=5, zorder=100)
        ax.autoscale(enable=False, axis='both')
        plt.show()


class GaussianMixtureModel2DViewer:
    """
    Class for viewing observation vector representaions within Gaussian Mixture Model.

    Attributes
    ----------
    kmm : KMeansModel
        Model to visualize.
    observation_vectors : list
        List of vectorized Observations.
    """
    def __init__(self, gmm, observation_vectors):
        """
        Parameters
        ----------
        gmm : GaussianMixtureModel
            Model to visualize.
        observation_vectors : list
            List of vectorized Observations.
        """
        self.gmm = gmm
        self.observation_vectors = observation_vectors

    def plot(self, x_index, y_index):
        """
        Generates 2D graph of distribution components of the of Gaussian Mixture model.

        Parameters
        ----------
        x_index : int
            Index of sample point to represent the x-axis value.
        y_index : int
            Index of sample point to represent the y-axis value.
        """
        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.gmm.n_components))]
        fig = plt.figure(figsize=(10, 10))
        ax = plt.subplot(111)

        X = self.observation_vectors
        labels = self.gmm.model.predict(self.observation_vectors)

        self.__make_ellipses(self.gmm.model, ax, colors)
        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            plt.scatter(data[:, x_index], data[:, y_index], s=0.8, color=color)
        plt.xticks(())
        plt.yticks(())
        plt.show()

    def __make_ellipses(self, gmm, ax, colors):
        """
        Generates and adds to the passed ax object ellipse figures.These ellipses represent component Gaussian
        distributions of the mixture model.

        Parameters
        ----------
        gmm : modeling.GaussianMixtureModel
            The gaussian mixture model for which to generate ellipses.
        ax : matplotlib.axes.Axes
            Subplot axes to which the ellipses are added.
        colors : list
            List of randomly generated colors used to differentiate the component distributions.
        """
        for n, color in enumerate(colors):
            if gmm.covariance_type == 'full':
                covariances = gmm.covariances_[n][:2, :2]
            elif gmm.covariance_type == 'tied':
                covariances = gmm.covariances_[:2, :2]
            elif gmm.covariance_type == 'diag':
                covariances = np.diag(gmm.covariances_[n][:2])
            elif gmm.covariance_type == 'spherical':
                covariances = np.eye(gmm.means_.shape[1]) * gmm.covariances_[n]
            v, w = np.linalg.eigh(covariances)
            u = w[0] / np.linalg.norm(w[0])
            angle = np.arctan2(u[1], u[0])
            angle = 180 * angle / np.pi  # convert to degrees
            v = 2. * np.sqrt(2.) * np.sqrt(v)
            ell = mpl.patches.Ellipse(gmm.means_[n, :2], v[0], v[1],
                                      180 + angle, color=color)
            ell.set_clip_box(ax.bbox)
            ell.set_alpha(0.5)
            ax.add_patch(ell)
