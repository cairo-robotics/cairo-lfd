import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib.colors import colorConverter
from mpl_toolkits.mplot3d import Axes3D
import randomcolor
import numpy as np


class SamplePointViewer:

    def __init__(self, sample_points):
        self.sample_points = sample_points

    def view_3D_scatter(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.autoscale_view()
        ax.scatter(self.sample_points[:, 0], self.sample_points[:, 1], self.sample_points[:, 2])
        plt.show()
        plt.close()  # Close a figure window


class KMeansModelViewer:

    def __init__(self, kmm, observations):
        self.kmm = kmm
        self.observations = observations

    def view_3D_clusters(self, x_index, y_index, z_index):
        """
        FRAGILE FUNCTION: DEPENDS ON THE ARRAY ELEMENTS BEING CORRECTLY MAPPED TO X,Y,Z OF THE OBSERVATIONS
        """
        labels = self.kmm.model.labels_
        centroids = self.kmm.model.cluster_centers_

        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.kmm.n_clusters))]

        fig = plt.figure(figsize=(10, 10))
        ax = Axes3D(fig)
        ax.autoscale_view()

        X = self.observations

        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            ax.scatter(data[:, x_index], data[:, y_index], data[:, z_index], color=color)
        ax.scatter(centroids[:, x_index], centroids[:, y_index], centroids[:, z_index], marker="x", s=150, linewidths=5, zorder=100)
        ax.autoscale(enable=False, axis='both')
        plt.show()


class GaussianMixtureModelViewer:

    def __init__(self, gmm, observations):
        self.gmm = gmm
        self.observations = observations

    def view_2D_gaussians(self, x_index, y_index):
        """
         FRAGILE FUNCTION: DEPENDS ON THE ARRAY ELEMENTS BEING CORRECTLY MAPPED TO X,Y OF THE OBSERVATIONS
        """
        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.gmm.n_components))]
        fig = plt.figure(figsize=(10, 10))
        ax = plt.subplot(111)

        X = self.observations
        labels = self.gmm.model.predict(self.observations)

        self._make_ellipses(self.gmm.model, ax, colors)
        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            plt.scatter(data[:, x_index], data[:, y_index], s=0.8, color=color)
        plt.xticks(())
        plt.yticks(())
        plt.show()

    def _make_ellipses(self, gmm, ax, colors):

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
