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

        """
        Import JSON file as a Python dictionary.

        Parameters
        ----------
        path : string
            Path of directory containing the .csv files.

        Returns
        -------
        entries : dict
            Dictionary representation of the JSON file.
        """

        with open(path, 'r') as f:
            datastore = json.load(f)
            return datastore

def load_json_files(path, count=None):
    """
    Import JSON files as a Python dictionary from .json files in the directory signified by the path..

    Parameters
    ----------
    path : string
        Path of directory containing the ..json files.

    Returns
    -------
    entries : dict
        Dictionary representation of the JSON file.
    """

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
    vectorized_demonstrations = []
    vectorized_demo = []
    for entry in demonstration:
        vector = entry['robot']['position']
        vectorized_demo.append(vector)
    vectorized_demo.reverse()
    vectorized_demonstrations.append(vectorized_demo)
    return vectorized_demonstrations


def plot_results(X, Y_, means, covariances, fig = plt.figure()):
    #fig = plt.figure()
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
        #print(radii)

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

class VGMMSegmentation():
    def __init__(self):
        pass

    def segment(self, data):

        print(data)
        print(len(data))
        print('\n')

        demo = vectorize_demonstration(data)

        print(demo)
        X = np.array([e for sl in demo for e in sl])
        print(np.shape(X))
        n_samples = len(X)
        print(X)

        vbgmm = VariationalGMM(n_components=n_samples).fit(X)

        prediction = vbgmm.predict(X)
        print(prediction)

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
        #print(currentnum)
        print(startindices)
        print(endindices)

        segments = []
        #print(data[3:3])
        #print(data[3])
        for index in range(len(startindices)):
            if startindices[index] == endindices[index]:
                print(data[startindices[index]:endindices[index]+1])
                segments.append(data[startindices[index]:endindices[index]+1])
            else:
                #(data[startindices[index]:endindices[index]])
                segments.append(data[startindices[index]:endindices[index]])

        plot_results(X, prediction, vbgmm.means_, vbgmm.covariances_)
        plt.title('Bayesian Gaussian Mixture Model on Shelf Placement Demonstration')
        plt.show()

        return segments

        #dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'place_atop_demos')
        #demos = vectorize_demonstrations(load_json_files(os.path.join(dir_path, '*.json'), count=5))
       
        #demolengths = []
        #for file in jsonfiles:
        #    print(os.path.join(dir_path,file))
        #    demo = load_json_files(os.path.join(dir_path,file))
        #    print(len(demo[0]))
        #    demolengths.append(len(demo[0]))
        #print(demolengths)

        #demos = vectorize_demonstrations(load_json_files(os.path.join(dir_path, '*.json'), count=5))
        #print(os.path.join(dir_path, '*.json'))
        #print(demos)

        #infile = 'all.json'
        #demos = vectorize_demonstrations(load_json_files(infile))
        #print(len(demos))
        #print(demos)
        #print('\n')

        #print('\n')
        #print(X[0][0])

        #print(X)

        #print("\n\nMy variational GMM")
        #print(vbgmm.means_)
        #print(vbgmm.covariances_)
        #print(vbgmm.mixture_density(X))
        #print(len(vbgmm.predict(X)))
        #for num in vbgmm.predict(X):
        #    print(num)

        #print(demolengths)
        #print(len(demolengths))
        #segloc = []
        #totallen = demolengths[0]
        #for endindices_ind in range(len(endindices)):
        #    print("yo")
        #    for demolengths_ind in range(len(demolengths)-1):
        #        print(demolengths_ind)
        #        if (endindices[endindices_ind] <= demolengths[demolengths_ind]):
        #            print("if")
        #            print(endindices[endindices_ind])
        #            print(demolengths[demolengths_ind])
        #            segloc.append(demolengths_ind)
        #            continue
        #        else:
        #            print("hi")
        #            totallen = totallen + demolengths[demolengths_ind+1] 
        #print(segloc)

        #plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_)
        #plt.title('Bayesian Gaussian Mixture Model on Shelf Placement Demonstration')
        #plt.show()


class ManualSegmentation():
    def __init__(self):
        pass

    def segment(self, data):

        print(data)

        # Initialze
        data[0]['segment'] = 1
        segchange = False  # track whether segment increment is necessary
        segments = [] # used to store each segment of data
        segstart = 0

        # Parse
        for i in range(1,len(data)): # iterate through blocks
            # iterate through items
            for j in range(len(data[i]['items'])): # iterate through items
                # iterate through in_contact
                for key in data[i]['items'][j]['in_contact']:
                    if data[i]['items'][j]['in_contact'][key] != data[i - 1]['items'][j]['in_contact'][key]:
                        segchange = True
            # iterate through in_contact for robot
            for key in data[i]['robot']['in_contact']:
                if data[i]['robot']['in_contact'][key] != data[i - 1]['robot']['in_contact'][key]:
                    segchange = True
            if data[i]['robot']['in_SOI'] != data[i - 1]['robot']['in_SOI']:
                segchange = True
            if segchange is True:
                data[i]['segment'] = data[i-1]['segment'] + 1
                segend = i-1
                segments.append(data[segstart:segend])
                segstart = i
                segchange = False
            else:
                data[i]['segment'] = data[i - 1]['segment']

        return segments


class Segment():
    def __init__(self, segments):
        self.segments = segments


class DemonstrationSegmenter():
    def __init__(self, segmenter):
        self.segmenter = segmenter

    def segment_demonstrations(self, demonstrations):
        allsegments = {}
        for i in range(len(demonstrations)):
            demo_id = i
            print(i)
            demo_segments = self.segmenter.segment(demonstrations[i])
            #print(demo_segments)
            #print('\n')
            allsegments[demo_id] = demo_segments
    	#print(allsegments)
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
            print("action")
            print(item)
            segobjects.append(Segment({key:temp[key] for key in [item]}))
            print({key:temp[key] for key in [item]})
        # SegmentationObject1.segments -> {
        #     1: segment1for1,
        #     2: segment1for2
        # }
        return [segobjects]

    def plot_segments(self, allsegments, segobjects):
        #print(allsegments[0])
        #print('\n')
        #print(allsegments[0][0])
        #print('\n')
        #print(allsegments[0][1])
        #print(allsegments[0][2])
        #data = allsegments[0][1]

        #print("pause")

        #demo = vectorize_demonstration(data)
        #X = np.array([e for sl in demo for e in sl])
        #vbgmm = VariationalGMM(n_components=len(data)).fit(X)
        #plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_)
        #plt.title('Bayesian Gaussian Mixture Model on Shelf Placement Demonstration')
        #plt.show()

        # Plot allsegments
        for demoind in range(len(allsegments)):
            for segind in range(len(allsegments[demoind])):   
                #data = allsegments[0][0]
                #print(data)
                #print(len(data))
                #data = segobjects[0][0]
                #print(segind)
                segment = allsegments[demoind][segind]
                print(segment)
                print(len(segment))
                if len(segment) > 1:
                    demo = vectorize_demonstration(segment)
                    #print(demo)
                    X = np.array([e for sl in demo for e in sl])
                    #print(X)
                    #print(np.shape(X))
                    vbgmm = VariationalGMM(n_components=len(segment)).fit(X)
                    if segind == 0:
                        fig = plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_)
                    else:
                        plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_,fig)
        plt.title('Bayesian Gaussian Mixture Model on Shelf Placement Demonstration')
        plt.show()
        # {
        #   1: [segment1for1, segment2for1, segment3for1],
        #   2: [segment1for2, segment2for2, segment3for2]
        # }         

        # Plot segobjects
        print("pause")
        #print(segobjects[0])
        #print(segobjects[0][0])
        #print(segobjects[0][0].segments)
        #for obj in segobjects:
        #    for demo in obj.segments:
        #        print(demo)
        #        print(segobj)
        #        for segment in segobj.segments:
        #            if len(segment) > 1:
        #                demo = vectorize_demonstration(segment)
                        #print(demo)
        #                X = np.array([e for sl in demo for e in sl])
                        #print(X)
                        #print(np.shape(X))
        #                vbgmm = VariationalGMM(n_components=len(segment)).fit(X)
        #                if segind == 0:
        #                    fig = plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_)
        #                else:
        #                    plot_results(X, vbgmm.predict(X), vbgmm.means_, vbgmm.covariances_,fig)
        #plt.title('Bayesian Gaussian Mixture Model on Shelf Placement Demonstration')
        #plt.show()                        

        # SegmentationObject1.segments -> {
        #     1: segment1for1,
        #     2: segment1for2
        # }

def main():
    print("starting")

    #infile = 'SHORTSOI.json'
    #outfile = 'OUT' + infile
    #importer = DataImporter()
    #exporter = DataExporter()
    #demo1 = load_json_file("all.json")
    #demo2 = demo1
    #demo3 = demo1
    #demonstrations = [demo1,demo2,demo3]

    #demo_segmenter = DemonstrationSegmenter(ManualSegmentation())
    #allsegments = demo_segmenter.segment_demonstrations(demonstrations)
    #segobjects = demo_segmenter.segment_classification(allsegments)

    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'place_atop_demos')
    demos = vectorize_demonstrations(load_json_files(os.path.join(dir_path, '*.json'), count=5))
    X = np.array([e for sl in demos for e in sl])
    vbgmm = VariationalGMM(n_components=10).fit(X)

    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'place_atop_demos')
    jsonfiles = [f for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))]
    print(jsonfiles)
    vgmm_segmenter = VGMMSegmentation()

    demonstrations = []
    for file in jsonfiles:
        #print(os.path.join(dir_path,file))
        data = load_json_file(os.path.join(dir_path,file))
        #print(data)
        demonstrations.append(data)
        #print('\n')
        #print(segments)
        #print(len(data[0]))

    demo_segmenter = DemonstrationSegmenter(VGMMSegmentation())
    #print(demonstrations)
    #print('\n')
    allsegments = demo_segmenter.segment_demonstrations(demonstrations)
    segobjects = demo_segmenter.segment_classification(allsegments)
    demo_segmenter.plot_segments(allsegments,segobjects)
    
    print("ending")

if __name__ == '__main__':
    main()