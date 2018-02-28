#!/usr/bin/env python

import rospy
import rospkg
import glob

import matplotlib.pyplot as plt
import numpy as np
import itertools
from matplotlib.pyplot import cm
from mpl_toolkits.mplot3d import Axes3D


from sawyer_interface.moveit_interface import  SawyerMoveitInterface

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Observation, Environment, import_configuration
from lfd_processor.data_io import DataImporter
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer, MotionPlanAnalyzer, TaskGraphAnalyzer, get_observation_pose_vector

def main():

    TRAJECTORIES = 0
    TRAJ_KEYFRAMES = 1
    CULLED_KEYFRAMES = 1

    #rospy.init_node("plot mak")
    print "hello world"

    '''constraint file path'''
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('lfd_processor_examples')
    config_loc = '/toy_data/configurations/'
    config_name = 'config.json'

    config_filepath = pkg_dir + config_loc + config_name


    '''create analyzers for plan evaluation'''
    configs = import_configuration(config_filepath)
    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])

    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()
    environment = Environment(items=None, robot=robot,
                              constraints=constraints)


    '''constraint file path'''
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('lfd_processor_examples')
    config_loc = '/toy_data/configurations/'
    config_name = 'config.json'

    config_filepath = pkg_dir + config_loc + config_name

    ''' original trajectory data'''
    traj_path = "/toy_data/full_system_test_two/"
    traj_files = glob.glob(pkg_dir + traj_path + "*.json")


    '''create data importer'''
    importer = DataImporter()



    ''' turn into array of dictionaries '''
    trajectories = []
    for traj_file in traj_files:
        trajectories.append(importer.import_json_to_dict(traj_file)["trajectories"][0])


    ''' create task graph files '''
    task_graph = TaskGraph(environment, "moveit_interface", get_observation_pose_vector)

    obsv_objects = []
    for traj_file in traj_files:
        trajecters = importer.import_json_to_dict(traj_file)["trajectories"][0]
        for obsv in trajecters:
            obsv_objects.append(Observation(obsv))

    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph() #step where head and tail are set
    task_graph.build_model(bandwidth=.008)
    task_graph.attribute_keyframe_type()
    task_graph_analyzer = TaskGraphAnalyzer(task_graph,"moveit_interface", get_observation_pose_vector)


    ''' create sample points '''
    for node in task_graph.get_keyframe_sequence():
        task_graph.sample_n_valid_waypoints(node, n=100, run_fk=False)


    ''' cull high liklihood nodes'''
    task_graph_analyzer.keyframe_culler(threshold=-1000)



    """""""""""""""""""""""""""""""""""""""""""""""""""
    Plotting section of code
    """""""""""""""""""""""""""""""""""""""""""""""""""
    ''' plot x and y of all trajectories '''
    if TRAJECTORIES:
        for index, trajectory in enumerate(trajectories):
            x_array = []
            y_array = []
            for point in trajectory:
                x, y, z = point["robot"]["position"]
                x_array.append(x)
                y_array.append(z)


            plt.plot(x_array, y_array)
        plt.show()



    '''show the keyframe grouping '''
    if TRAJ_KEYFRAMES:
        color = cm.rainbow(np.linspace(0, 1, 55))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        print color[1]
        print len(trajectories)
        for index, trajectory in enumerate(trajectories):
            data_tuple_array = []
            for i, point in enumerate(trajectory):
                keyframe_id = point["keyframe_id"]
                keyframe_type = point["keyframe_type"]
                pose_data = point["robot"]["position"]
                pose_data += point["robot"]["orientation"]

                data_tuple_array.append((keyframe_id, keyframe_type, pose_data))

            data_1 = []
            data_2 = []
            data_3 = []
            keyframe = 1
            for data in data_tuple_array:
                if data[0] is not None:
                    if keyframe is not data[0]:
                        ax.scatter(data_1, data_2, data_3, marker='o', c=color[keyframe])
                        data_1 = []
                        data_2 = []
                        data_3 = []
                        keyframe += 1
                    else:
                        data_1.append(data[2][0])
                        data_2.append(data[2][1])
                        data_3.append(data[2][2])
        plt.show()


    if CULLED_KEYFRAMES:
        color = cm.rainbow(np.linspace(0, 1, 56))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for node in task_graph.get_keyframe_sequence():
            print node
            node_samples = task_graph.nodes[node]["samples"]
            data_1 = []
            data_2 = []
            data_3 = []
            for sample in node_samples:
                pose_list = sample.get_pose_list()
                data_1.append(pose_list[0])
                data_2.append(pose_list[1])
                data_3.append(pose_list[2])
            ax.scatter(data_1, data_2, data_3, c=color[node])

        plt.show()

    return 0










if __name__ == '__main__':
    rospy.init_node("plotting_ode")
    main()
