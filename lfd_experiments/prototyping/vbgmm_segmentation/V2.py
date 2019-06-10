import numpy as np
from lfd.data_io import DataImporter, DataExporter

import os
import json
from collections import OrderedDict
import glob
import codecs
import itertools
import errno
import random
from vbgmm_precision import VariationalGMM
from sklearn import mixture

import numpy as np
from scipy import linalg
import matplotlib
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization!

color_iter = itertools.cycle(['navy', 'black', 'cornflowerblue', 'r',
                              'darkorange', 'g', 'brown'])

def load_json_file(path):
        # Load a single file with one function call
        with open(path, 'r') as f:
            datastore = json.load(f)
            return datastore

def load_json_files(path, count=None):
    # Load multiple files with one function call
    entries = []
    files = glob.glob(path)
    if count is not None and count > 0:
        files = files[0:count]
    for name in files:
        try:
            with codecs.open(name, "r", 'utf-8') as f:
                file_data = json.load(f, object_pairs_hook=OrderedDict)
                entries.append(file_data)
        except IOError as exc:
            if exc.errno != errno.EISDIR:
                raise  # Propagate other kinds of IOError.
    return entries

def vectorize_demonstrations(demonstrations):
    # Retrieve position coordinates for multiple demonstrations
    vectorized_demonstrations = []
    for demo in demonstrations:
        vectorized_demo = []
        for entry in demo:
            vector = entry['robot']['position']
            vectorized_demo.append(vector)
        vectorized_demo.reverse()
        vectorized_demonstrations.append(vectorized_demo)
    return vectorized_demonstrations

def vectorize_demonstration(demonstration):
    # Retrieve position coordinates for a single demonstration
    vectorized_demonstrations = []
    vectorized_demo = []
    for entry in demonstration:
        vector = entry['robot']['position']
        vectorized_demo.append(vector)
    vectorized_demo.reverse()
    vectorized_demonstrations.append(vectorized_demo)
    return vectorized_demonstrations

def plot_results(X, Y_, means, covariances, fig = plt.figure()):
    ax = fig.add_subplot(111, projection='3d')
    for i, (mean, covar, color) in enumerate(zip(
            means, covariances, color_iter)):

        # as the DP will not use every component it has access to
        # unless it needs it, we shouldn't plot the redundant
        # components.
        if not np.any(Y_ == i):
            continue
        ax.scatter(X[Y_ == i, 0], X[Y_ == i, 1], X[Y_ ==i, 2], s=.8, color=color)

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
    return ax

class VBGMMSegmentation():
    def __init__(self, alldemos):
        self.alldemos = alldemos
        self.build_model(alldemos)

    def build_model(self, data):
        # Build model using every observation available
        demos = vectorize_demonstrations(data)
        X = np.array([e for sl in demos for e in sl])
        n_samples_tot = len(X)
        print(n_samples_tot)
        if n_samples_tot < 10:
           self.vbgmm = VariationalGMM(n_components=n_samples_tot).fit(X)
        else:
           self.vbgmm = VariationalGMM(n_components=10).fit(X)
        plot_results(X, self.vbgmm.predict(X), self.vbgmm.means_, self.vbgmm.covariances_)
        plt.show()

    def segment(self, data):
        # Predict segmentation using trained model
        demo = vectorize_demonstration(data)
        X = np.array([e for sl in demo for e in sl])
        n_samples = len(X)
        print(X)
        prediction = self.vbgmm.predict(X)

        # Find start and end indices for each observation
        startindices = []
        endindices = []
        for i in range(len(prediction)):
            if i == 0:
                currentnum = prediction[i]
                startindices.append(i)
            elif (prediction[i] != currentnum) & (i == len(prediction)-1):
                endindices.append(i-1)
                startindices.append(i)
                endindices.append(i)
                currentnum = prediction[i]
            elif (prediction[i] != currentnum):
                endindices.append(i-1)
                startindices.append(i)
                currentnum = prediction[i]
            elif i == len(prediction)-1:
                endindices.append(i)

        # Use start and end indices to create/splice segments
        segments = []
        for index in range(len(startindices)):
            if startindices[index] == endindices[index]:
                segments.append(data[startindices[index]:endindices[index]+1])
            else:
                segments.append(data[startindices[index]:endindices[index]])

        # Return
        return segments        

class Segment():
    def __init__(self,segments):
        self.segments = segments

class DemonstrationSegmenter():
    def __init__(self, segmenter):
        self.segmenter = segmenter

    def segment_demonstrations(self, demonstrations):
        allsegments = {}
        for i in range(len(demonstrations)):
            demo_id = i
            demo_segments = self.segmenter.segment(demonstrations[i])
            allsegments[demo_id] = demo_segments
        # {
        #   1: [segment1for1, segment2for1, segment3for1],
        #   2: [segment1for2, segment2for2, segment3for2]
        # }    		
    	return allsegments

    def segment_classification(self, allsegments):
        temp = {}
        for demonum in range(len(allsegments)):
            for segnum in range(len(allsegments[demonum])):
                if segnum not in temp:
                    temp[segnum] = {demonum:allsegments[demonum][segnum]}
                else:
                    temp[segnum][demonum] = allsegments[demonum][segnum]

        segobjects = []
        for item in temp:
            segobjects.append(Segment({key:temp[key] for key in [item]}))
        # SegmentationObject1.segments -> {
        #     1: segment1for1,
        #     2: segment1for2
        # }
        return [segobjects]

    def plot_segments(self, allsegments, segobjects, vbgmm):
        # Plot allsegments
        for demoind in range(len(allsegments)):
            for segind in range(len(allsegments[demoind])):   
                segment = allsegments[demoind][segind]
                if len(segment) > 1:
                    demo = vectorize_demonstration(segment)
                    X = np.array([e for sl in demo for e in sl])
                    if segind == 0:
                        fig = plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_)
                    else:
                        plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_,fig)
        plt.show()
        # {
        #   1: [segment1for1, segment2for1, segment3for1],
        #   2: [segment1for2, segment2for2, segment3for2]
        # }         

def main():
    # Build model
    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'test_data')
    alldemos = load_json_files(os.path.join(dir_path, '*.json'), count=5)
    vbgmm_segmenter = VBGMMSegmentation(alldemos)

    # Deploy model to segment each demonstration
    demo_segmenter = DemonstrationSegmenter(vbgmm_segmenter)
    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'test_data')
    jsonfiles = [f for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))]
    demonstrations = []
    for file in jsonfiles:
        data = load_json_file(os.path.join(dir_path,file))
        demonstrations.append(data)
    allsegments = demo_segmenter.segment_demonstrations(demonstrations)
    segobjects = demo_segmenter.segment_classification(allsegments)

    # Plot segments
    demo_segmenter.plot_segments(allsegments,segobjects)
    
    print("ending")

if __name__ == '__main__':
    main()