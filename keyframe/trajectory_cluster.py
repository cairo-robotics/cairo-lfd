import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture
import matplotlib as mpl
from matplotlib import pyplot as plt
from matplotlib.colors import colorConverter
from mpl_toolkits.mplot3d import Axes3D
from data_processing import DataProcessor, DataImporter
import randomcolor


class KMeansClusterer:

    def __init__(self, observations, n_clusters=5):
        self.observations = observations
        self.n_clusters = n_clusters
        self.kmeans = KMeans(n_clusters=self.n_clusters)

    def kmeans_fit(self):
        self.kmeans.fit(self.observations)

    def get_cluster_samples(self):
        cluster_data = {}
        labels = self.kmeans.labels_
        for i in range(0, self.n_clusters):
            cluster_data[i+1] = [self.observations[np.where(labels == i)]]
        return cluster_data


    def view_XYZ_clusters(self, x_index, y_index, z_index):
        """
        FRAGILE FUNCTION: DEPENDS ON THE ARRAY ELEMENTS BEING CORRECTLY MAPPED TO X,Y,Z OF THE OBSERVATIONS
        """
        labels = self.kmeans.labels_
        centroids = self.kmeans.cluster_centers_

        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.n_clusters))]

        fig = plt.figure(figsize=(10, 10))
        ax = Axes3D(fig)
        ax.autoscale_view()

        X = self.observations

        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            ax.scatter(data[:, x_index], data[:, y_index], data[:, z_index], color=color)
        ax.scatter(centroids[:, x_index], centroids[:, y_index], centroids[:, z_index], marker="x", s=150, linewidths=5, zorder=100)
        ax.autoscale(enable=False, axis='both')  # you will need this line to change the Z-axis

        plt.show()
        plt.close()  # Close a figure window


class GMMKeyframe:

    def __init__(self, observations, n_components = 5, means_init=None):
        self.observations = observations
        self.n_components = n_components
        self.gmm = GaussianMixture(n_components=self.n_components, covariance_type='full', means_init=means_init)

    def gmm_fit(self):
        self.gmm.fit(self.observations)

    def view_2D_gaussians(self, x_index, y_index):
        """
         FRAGILE FUNCTION: DEPENDS ON THE ARRAY ELEMENTS BEING CORRECTLY MAPPED TO X,Y OF THE OBSERVATIONS
        """
        rand_color = randomcolor.RandomColor()
        colors = [colorConverter.to_rgb(rand_color.generate()[0]) for n in enumerate(range(0, self.n_components))]
        fig = plt.figure(figsize=(10, 10))
        ax = plt.subplot(111)

        X = self.observations
        labels = self.gmm.predict(self.observations)

        self._make_ellipses(self.gmm, ax, colors)
        for n, color in enumerate(colors):
            data = X[np.where(labels == n)]
            plt.scatter(data[:, x_index], data[:, y_index], s=0.8, color=color)
        plt.xticks(())
        plt.yticks(())
        plt.show()

    def generate_samples(self, n_samples):
        points, labels = gmm_keyframer.gmm.sample(n_samples)
        return points

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


if __name__ == "__main__":

    importer = DataImporter()
    processor = DataProcessor()

    trajectories = importer.import_csv_to_list('../toy_data/*.csv')
    observations = processor.concatenate_trajectory_observations(trajectories)
    observations = [[entry[1], entry[2], entry[3], entry[4], entry[5], entry[6], entry[7]] for entry in observations]
    np_observation = processor.convert_to_numpy(observations)

    km_clusterer = KMeansClusterer(np_observation, n_clusters=5)
    km_clusterer.kmeans_fit()
    km_clusterer.view_XYZ_clusters()
    cluster_data = km_clusterer.get_cluster_samples()

    counter = 0
    for cluster_number, np_array in cluster_data.items():
        gmm_keyframer = GMMKeyframe(np_array[0])
        gmm_keyframer.gmm_fit()
        gmm_keyframer.view_2D_gaussians()
        points, labels = gmm_keyframer.gmm.sample(500)
        samples = list(zip(points, labels))
        print(samples)
        sample_points = np.array([sample[0] for sample in samples])
        counter += 1
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.autoscale_view()
        ax.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2])
        plt.show()
        plt.close()  # Close a figure window