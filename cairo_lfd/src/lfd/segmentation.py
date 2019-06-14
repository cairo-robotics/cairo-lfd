import rospy
import numpy as np
import copy
from sklearn import mixture
from lfd.data_conversion import vectorize_demonstration


class Segment():
    def __init__(self, demo_id, observations):
        self.demo_id = demo_id
        self.observations = observations


class DemonstrationSegmentation():
    def __init__(self, segmenter):
        self.segmenter = segmenter

    def segment_demonstrations(self, demonstrations):
        all_segments = {}
        for i in range(len(demonstrations)):
            demo_id = i
            demo_segments = self.segmenter.segment(demonstrations[i])
            all_segments[demo_id] = demo_segments
        segments = self._segment_classification(all_segments)
        return segments

    def _segment_classification(self, all_segments):
        temp = {}
        for demo_num in range(len(all_segments)):
            for seg_num in range(len(all_segments[demo_num])):
                if seg_num not in temp:
                    temp[seg_num] = {demo_num: all_segments[demo_num][seg_num]}
                else:
                    temp[seg_num][demo_num] = all_segments[demo_num][seg_num]
        seg_objects = []
        for item in temp:
            seg_objects.append(Segment({key: temp[key] for key in [item]}))
        return [seg_objects]


class GMMSegmenter():

    def __init__(self, demonstrations, demonstration_vectorizor, n_components):
        self.demos = demonstrations
        self.vectorizor = demonstration_vectorizor
        self.n_components = n_components
        self.n_samples = len(demonstrations)
        self._fit_model()

    def _fit_model(self):
        # Build model using every observation available
        demos = [self.vectorizor(demo) for demo in self.demos]
        X = np.array([e for sl in demos for e in sl])
        if self.n_samples < self.n_components:
            self.model = mixture.GaussianMixture(n_components=X.shape[0]).fit(X)
        else:
            self.model = mixture.GaussianMixture(n_components=n_components).fit(X)

    def segment(self, demonstration):
        # Predict segmentation using trained model
        demo = np.array(self.vectorizor(demonstration))
        X = np.array(demo)
        prediction = self.model.predict(X)

        # Find start and end indices for each observation
        startindices = []
        endindices = []
        for i in range(len(prediction)):
            if i == 0:
                currentnum = prediction[i]
                startindices.append(i)
            elif (prediction[i] != currentnum) & (i == len(prediction) - 1):
                endindices.append(i - 1)
                startindices.append(i)
                endindices.append(i)
                currentnum = prediction[i]
            elif (prediction[i] != currentnum):
                endindices.append(i - 1)
                startindices.append(i)
                currentnum = prediction[i]
            elif i == len(prediction) - 1:
                endindices.append(i)

        # Use start and end indices to create/splice segments
        segments = []
        for index in range(len(startindices)):
            if startindices[index] == endindices[index]:
                segments.append(
                    X[startindices[index]:endindices[index] + 1])
            else:
                segments.append(X[startindices[index]:endindices[index]])

        # Return
        return segments


class LabelBasedSegmenter():
    def __init__(self):
        pass

    def segment(self, observations):

        # Initialze
        observations.data[0]['segment'] = 1
        segchange = False  # track whether segment increment is necessary

        # Parse
        for i in range(1, len(observations)):  # iterate through blocks
            # iterate through items
            for j in range(len(observations[i].data['items'])):
                # iterate through in_contact
                for key in observations.data[i]['items'][j]['in_contact']:
                    if observations.data[i]['items'][j]['in_contact'][key] != observations.data[i - 1]['items'][j]['in_contact'][key]:
                        segchange = True
            # iterate through in_contact for robot
            for key in observations.data[i]['robot']['in_contact']:
                if observations.data[i]['robot']['in_contact'][key] != observations.data[i - 1]['robot']['in_contact'][key]:
                    segchange = True
            if observations.data[i]['robot']['in_SOI'] != observations.data[i - 1]['robot']['in_SOI']:
                segchange = True
            if segchange is True:
                observations.data[i]['segment'] = observations.data[i -
                                                                    1]['segment'] + 1
                segchange = False
            else:
                observations.data[i]['segment'] = observations.data[i - 1]['segment']
