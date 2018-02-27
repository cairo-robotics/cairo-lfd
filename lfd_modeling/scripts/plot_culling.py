#!/usr/bin/env python

import rospy
import rospkg
import glob

import matplotlib.pyplot as plt
import numpy as np

from sawyer_interface.moveit_interface import  SawyerMoveitInterface

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Observation, Environment, import_configuration
from lfd_processor.data_io import DataImporter
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer, MotionPlanAnalyzer, TaskGraphAnalyzer

def main():

    #rospy.init_node("plot mak")
    print "hello world"


    '''constraint file path'''
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('lfd_processor_examples')
    config_loc = '/toy_data/configurations/'
    config_name = 'config.json'

    config_filepath = pkg_dir + config_loc + config_name

    ''' original trajectory data'''
    traj_path = "/toy_data/full_system_test/"
    traj_files = glob.glob(pkg_dir + traj_path + "*.json")


    '''create data importer'''
    importer = DataImporter()



    ''' turn into array of dictionaries '''
    trajectories = []
    for traj_file in traj_files:
        trajectories.append(importer.import_json_to_dict(traj_file)["trajectories"][0])



    ''' plot x and y of all trajectories '''
    for index, trajectory in enumerate(trajectories):
        x_array = []
        y_array = []
        for point in trajectory:
            x, y, z = point["robot"]["position"]
            x_array.append(x)
            y_array.append(y)


        plt.plot(x_array, y_array)
    plt.show()




    ''' show the keyframe grouping '''
    
    return 0

























if __name__ == '__main__':
    main()
