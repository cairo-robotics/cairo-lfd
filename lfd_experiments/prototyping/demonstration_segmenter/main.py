import numpy as np
from lfd.data_io import DataImporter, DataExporter

class ManualSegmentation():
    def __init__(self):
        pass

    def segment(self, data):

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
        for i in range(len(allsegments)):
            for j in range(len(allsegments[i])):
                if j not in temp:
                    temp[j] = {i:allsegments[i][j]}
                else:
                    temp[j][i] = allsegments[i][j]

        segobjects = []
        for item in temp:
            segobjects.append(Segment(item))
        # SegmentationObject1.segments -> {
        #     1: segment1for1,
        #     2: segment1for2
        # }
        
        return [segobjects]

def main():
    print("starting")

    infile = 'SHORTSOI.json'
    outfile = 'OUT' + infile
    importer = DataImporter()
    exporter = DataExporter()
    demo1 = importer.load_json_file(infile)
    demo2 = demo1
    demo3 = demo1
    demonstrations = [demo1,demo2,demo3]

    demo_segmenter = DemonstrationSegmenter(ManualSegmentation())
    allsegments = demo_segmenter.segment_demonstrations(demonstrations)
    segobjects = demo_segmenter.segment_classification(allsegments)

    print("ending")

if __name__ == '__main__':
    main()