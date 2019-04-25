#!/usr/bin/env python

import argparse
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface

from modeling.graphing import ObservationClusterer, KeyframeGraph
from modeling.models import KDEModel
from modeling.sampling import KeyframeSampler
from lfd.environment import Demonstration, Observation, Environment, import_configuration
from lfd.data_io import DataImporter
from lfd.items import ItemFactory
from lfd.constraints import ConstraintFactory
from lfd.analysis import KeyframeGraphAnalyzer, ConstraintAnalyzer
from lfd.conversion import SawyerSampleConverter, get_observation_joint_vector
from lfd_experiments.srv import *

class PerformDemonstrationServer():

    def __init__(self):
        rospy.init_node('perform_demonstration_server')

    def run(self):
        service = rospy.Service("perform_demonstration", PerformDemonstration, self.handle)
        rospy.spin()

    def handle(self, constraint_types):
        # Get the params from ROS
        config = rospy.get_param("CONFIG")
        directory = rospy.get_param("DIRECTORY")
        bandwidth = rospy.get_param("BANDWIDTH")
        threshold = rospy.get_param("THRESHOLD")
        number_of_samples = rospy.get_param("NUMBER_OF_SAMPLES")

        # Import the data
        importer = DataImporter()
        labeled_demonstrations = importer.load_json_files(directory + "/*.json")

        # Convert imported data into Demonstrations and Observations
        demonstrations = []
        for datum in labeled_demonstrations["data"]:
            observations = []
            for entry in datum:
                observations.append(Observation(entry))
            demonstrations.append(Demonstration(observations))

        if len(demonstrations) == 0:
            rospy.logwarn("CC-LfD: No demonstration data to model!!")
            return 0

        rospy.init_node("graph_traverse")

        """ Create the Cairo LfD environment """
        config_filepath = config
        # NOTE: the config file can be used to parameterize any of the constraints we want to add to the learned skill
        configs = import_configuration(config_filepath)
        items = ItemFactory(configs).generate_items()
        constraints = ConstraintFactory(configs).generate_constraints()
        # We only have just the one robot...for now.......
        environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints)

        """ Create the moveit_interface """
        moveit_interface = SawyerMoveitInterface()
        moveit_interface.set_velocity_scaling(.35)
        moveit_interface.set_acceleration_scaling(.25)

        """ Create KeyframeGraph object. """
        graph = KeyframeGraph()
        cluster_generator = ObservationClusterer()

        """
        Generate clusters using labeled observations, build the models, graphs, and atributes for each
        cluster in the KeyFrameGraph
        """
        clusters = cluster_generator.generate_clusters(demonstrations)
        for cluster_id in clusters.keys():
            graph.add_node(cluster_id)
            graph.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            graph.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
            graph.nodes[cluster_id]["applied_constraints"] = clusters[cluster_id]["applied_constraints"]
            graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=bandwidth)
        graph.add_path(graph.nodes())
        graph.fit_models(get_observation_joint_vector)
        rospy.loginfo(graph.get_keyframe_sequence())
        for node in graph.get_keyframe_sequence():
            print(graph.nodes[node]["keyframe_type"])

        # NOTE: Graph has been made so we can simply edit applied_constraints field to add our new constraints for sampling
        # NOTE: Need pkgs: cairo_lfd, cairo_robot_interface, constraint_classification

        """ Build a ConstraintAnalyzer and KeyframeGraphAnalyzer """
        constraint_analyzer = ConstraintAnalyzer(environment)
        graph_analyzer = KeyframeGraphAnalyzer(graph, moveit_interface, get_observation_joint_vector)

        sample_to_obsv_converter = SawyerSampleConverter(moveit_interface)
        sampler = KeyframeSampler(constraint_analyzer, sample_to_obsv_converter)

        """ Generate raw_samples from graph for each keyframe """
        for node in graph.get_keyframe_sequence():
            # Keep sampling 
            if graph.nodes[node]["keyframe_type"] == "constraint_transition":
                rospy.loginfo("Sampling from a constraint transition keyframe.")
                attempts, samples, matched_ids = sampler.generate_n_valid_samples(graph.nodes[node]["model"], graph.nodes[node]["applied_constraints"], n=n_samples)
                if len(samples) == 0:
                    # Some constraints couldn't be sampled successfully, so using best available samples.
                    diff = list(set(graph.nodes[node]["applied_constraints"]).difference(set(matched_ids)))
                    rospy.logwarn("Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    attempts, samples, matched_ids = sampler.generate_n_valid_samples(graph.nodes[node]["model"], matched_ids, n=n_samples)

            else:
                n_samples = number_of_samples
                attempts, samples, matched_ids = sampler.generate_n_valid_samples(graph.nodes[node]["model"], graph.nodes[node]["applied_constraints"], n=n_samples)

            rospy.loginfo("Keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < n_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), n_samples)
            if len(samples) == 0:
                rospy.loginfo("Keyframe %d has no valid sample observations", node)
                graph.cull_node(node)
            # Order sampled points based on their intramodel log-liklihood
            ranked_samples = sampler.rank_samples(graph.nodes[node]["model"], samples)

            # User converter object to conver raw sample vectors into LfD observations
            graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(sample, run_fk=True) for sample in ranked_samples]

        """ Clear occluded points (points in collision etc,.) """
        for node in graph.get_keyframe_sequence():
            samples = graph.nodes[node]["samples"]
            free_samples, trash = graph_analyzer.evaluate_keyframe_occlusion(samples)
            if free_samples == []:
                rospy.loginfo("Keyframe {} has no free samples and will be culled.".format(node))
                graph.cull_node(node)
            else:
                graph.nodes[node]["free_samples"] = free_samples

        """ Cull/remove keyframes/nodes that via change point estimation using log-liklihood """
        graph_analyzer.cull_keyframes(threshold=threshold)

        # """ Order sampled points based on their intramodel log-liklihood """
        # for node in graph.get_keyframe_sequence():
        #     graph.rank_waypoint_samples(node)

        rospy.loginfo(graph.get_keyframe_sequence())

        """ Create a seequence of keyframe waypoints and excute motion plans to reconstruct skill """
        joint_config_array = []
        for node in graph.get_keyframe_sequence():
            sample = graph.nodes[node]["free_samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

        moveit_interface.move_to_joint_targets(joint_config_array)

        return True

if __name__ == '__main__':
    try:
        obj = PerformDemonstrationServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass
