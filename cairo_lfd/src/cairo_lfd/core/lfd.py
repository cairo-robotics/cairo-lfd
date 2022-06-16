import random

import rospy
import networkx as nx

from cairo_lfd_msgs.msg import NodeTime
from cairo_lfd.core.environment import Environment
from cairo_lfd.constraints.triggers import TriggerFactory
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.core.robots import RobotFactory
from cairo_lfd.data.conversion import SawyerSampleConversion
from cairo_lfd.data.vectorization import get_observation_joint_vector
from cairo_lfd.data.io import export_to_json
from cairo_lfd.constraints.acc_assignment import assign_autoconstraints, AutoconstraintFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.constraints.optimizers import OptimizerFactory
from cairo_lfd.modeling.graphing import KeyframeClustering, KeyframeGraph, IntermediateTrajectories
from cairo_lfd.modeling.models import KDEModel
from cairo_lfd.modeling.sampling import KeyframeModelSampler, ModelScoreRanking, ConfigurationSpaceRanking, AutoconstraintSampler
from cairo_lfd.modeling.analysis import get_culling_candidates, check_constraint_validity, check_state_validity

from cairo_lfd_msgs.msg import AppliedConstraints


class ACC_LFD():

    def __init__(self, configs, modeling_settings, robot_interface):
        self.configs = configs
        self.settings = modeling_settings
        self.robot_interface = robot_interface

    def build_environment(self):
        robots = RobotFactory(self.configs['robots']).generate_robots()
        items = ItemFactory(self.configs['items']).generate_items()
        triggers = TriggerFactory(self.configs['triggers']).generate_triggers()
        constraints = ConstraintFactory(
            self.configs["constraints"]).generate_constraints()
        # We only have just the one robot...for now.......
        self.environment = Environment(
            items=items, robot=robots[0], constraints=constraints, triggers=triggers)

    def build_keyframe_graph(self, demonstrations, bandwidth, vectorizor=None):
        self.G = KeyframeGraph()
        keyframe_clustering = KeyframeClustering()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = keyframe_clustering.get_clusters(demonstrations)
        for cluster_id in clusters.keys():
            self.G.add_node(cluster_id)
            self.G.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.G.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]

            ####################################################
            # NEED TO BE ABLE TO SUPPORT CC-LFD eventually!
            # graph.nodes[cluster_id]["applied_constraints"] = [clusters[cluster_id]["applied_constraints"]]
            self.G.nodes[cluster_id]["applied_constraints"] = []
            ####################################################
            self.G.nodes[cluster_id]["autoconstraints"] = {}
            # Used to track changes in the autoconstraint assignment according to segmentation and proximity style constraint assignment.
            self.G.nodes[cluster_id]["autoconstraint_transitions"] = []
            self.G.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        nx.add_path(self.G, self.G.nodes())
        if vectorizor is not None:
            self.G.fit_models(vectorizor)
        else:
            self.G.fit_models(get_observation_joint_vector)
        self.G.identify_primal_observations(get_observation_joint_vector)

    def generate_autoconstraints(self, demonstrations):
        rospy.loginfo("Building autoconstraints.")
        # Encapsulates all segmentation, heuristic modeling:
        autoconstraint_builders = AutoconstraintFactory(
            self.configs).generate_autoconstraint_builders(demonstrations)
        # Assigns built autosconstraints to keyframes nodes.
        assign_autoconstraints(self.G, autoconstraint_builders)
        self._apply_autoconstraint_transitions()

    def _apply_autoconstraint_transitions(self):
        prev_set = set()
        for node in self.G.get_keyframe_sequence():
            curr_set = set(self.G.nodes[node]
                           ["autoconstraint_transitions"])
            diff = prev_set.symmetric_difference(curr_set)
            if len(diff) > 0:
                self.G.nodes[node]["keyframe_type"] = "constraint_transition"
            prev_set = curr_set

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=-1000):
        number_of_samples = self.settings.get(
            "number_of_samples", number_of_samples)
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.G.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, validated_set, autoconstraint_attempts, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            if autoconstraint_attempts == 10:
                rospy.logwarn(
                    "Not able to sample enough points for autoconstrained keyframe {}.".format(node))
                self.G.cull_node(node)
                continue
            rospy.loginfo("Validated autoconstraints: {}".format(
                list(validated_set)))
            rospy.loginfo("Keyframe %d: %s valid of %s attempts",
                          node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.G.cull_node(node)
                continue
            self.G.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.G.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, samples, prior_sample)
            self.G.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.G, automate_threshold, culling_threshold):
            self.G.cull_node(node)

    def _generate_samples(self, node, keyframe_sampler, number_of_samples, min_samples=5, constraint_attempts=10):
        validated_set = set()
        attempted_count = 0
        autoconstraint_sampler = AutoconstraintSampler(
            self.G.nodes[node]["autoconstraints"])
        valid_samples = []
        constraints = []
        autoconstraint_attempts = 0

        while autoconstraint_sampler.validate(validated_set) is False or len(valid_samples) < min_samples:
            if autoconstraint_attempts == constraint_attempts:
                break
            autoconstraint_attempts += 1
            constraints = autoconstraint_sampler.sample(validated_set)
            attempted_count, samples, validated_set = keyframe_sampler.sample(self.environment,
                                                                              self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            valid_samples.extend(samples)
        return attempted_count, valid_samples, validated_set, autoconstraint_attempts, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.G.fit_models_on_valid_samples(
            node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.sample(self.environment,
                                                               self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.G.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.G.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """
        joint_config_array = []
        for node in self.G.get_keyframe_sequence():
            sample = self.G.nodes[node]["samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

        self.robot_interface.move_to_joint_targets(joint_config_array)


class CC_LFD_OPT():

    def __init__(self, configs, model_settings, robot_interface):
        self.configs = configs
        self.settings = model_settings
        self.cull_overconstrained = self.settings.get(
            "cull_overconstrained", True)
        self.robot_interface = robot_interface

    def build_environment(self):
        robots = RobotFactory(self.configs['robots']).generate_robots()
        items = ItemFactory(self.configs['items']).generate_items()
        triggers = TriggerFactory(self.configs['triggers']).generate_triggers()
        constraints = ConstraintFactory(
            self.configs['constraints']).generate_constraints()
        optimizers = OptimizerFactory(self.configs['optimizers']).generate_optimizers()
        # We only have just the one robot...for now.......
        self.environment = Environment(
            items=items, robot=robots[0], constraints=constraints, optimizers=optimizers, triggers=triggers)

    def build_keyframe_graph(self, labeled_demonstrations, bandwidth):
        self.G = KeyframeGraph()
        self.G.graph['labeled_demonstrations'] = labeled_demonstrations
        self.G.graph['intermediate_trajectories'] = IntermediateTrajectories(
        ).get_trajectories(labeled_demonstrations)
        keyframe_clustering = KeyframeClustering()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = keyframe_clustering.get_clusters(labeled_demonstrations)
        for cluster_id in clusters.keys():
            self.G.add_node(cluster_id)
            self.G.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.G.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
            self.G.nodes[cluster_id]["applied_constraints"] = clusters[cluster_id]["applied_constraints"]
            self.G.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        nx.add_path(self.G, self.G.nodes())
        self.G.fit_models(get_observation_joint_vector)
        self.G.identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=5, use_optimizers=False):
        culling_threshold = self.settings.get("culling_threshold", culling_threshold)
        
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.G.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, generated_samples, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples, use_optimizers)
            rospy.loginfo(
                "Initial sampling: %s valid of %s attempts", len(generated_samples), attempts)
            if len(generated_samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    generated_samples), number_of_samples)
            if len(generated_samples) == 0:
                if self.cull_overconstrained:
                    rospy.logwarn("No valid samples found so culling keyframe from the model (this is permanent).")
                    self.G.cull_node(node)
                else:
                    rospy.logwarn("No valid samples found. Original keyframe will be kept but will likely be constraint non-compliant.")
                continue
            self.G.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in generated_samples]
            self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            attempts, generated_samples, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples, use_optimizers)
            rospy.loginfo(
                "Refitted keyframe: %s valid of %s attempts", len(generated_samples), attempts)
            if len(generated_samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    generated_samples), number_of_samples)
            if len(generated_samples) == 0:
                if self.cull_overconstrained:
                    rospy.logwarn("No valid samples found when refitting model so culling keyframe from the model (this is permanent).")
                    self.G.cull_node(node)
                else:
                    rospy.logwarn("No valid samples found from refitted model. Original keyframe will be kept but will likely be constraint non-compliant.")
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, generated_samples, prior_sample)
            self.G.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.G, automate_threshold, culling_threshold):
            self.G.cull_node(node)

    def _generate_samples(self, node, sampler, number_of_samples, use_optimizers=False):
        optimizer = self.environment.get_optimizers_by_ids(list(self.G.nodes[node]["applied_constraints"]))
        if self.G.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            if use_optimizers and optimizer is not None:
                rospy.loginfo("Generating samples using optimizer.")
                constraints = [self.environment.get_constraint_by_id(
                    constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                unconstrainted_keyframe_samples = sampler.sample(self.environment, self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], [], n=number_of_samples)
                optimized_samples_raw = [optimizer.optimize(point, point) for point in unconstrainted_keyframe_samples]
                attempts = len(optimized_samples_raw)
                samples = [sample for sample in optimized_samples_raw if sample is not None]
                matched_ids = list(self.G.nodes[node]["applied_constraints"]) 
            else:
                constraints = [self.environment.get_constraint_by_id(
                    constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                if len(samples) == 0:
                    # Some constraints couldn't be sampled successfully, so using best available samples.
                    diff = list(set(self.G.nodes[node]["applied_constraints"]).difference(
                        set(matched_ids)))
                    if len(matched_ids) > 0:
                        rospy.logwarn(
                            "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                        constraints = [self.environment.get_constraint_by_id(
                            constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                        attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                        self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                    else:
                        rospy.logwarn(
                            "Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            if use_optimizers and optimizer is not None:
                rospy.loginfo("Generating samples using optimizer.")
                constraints = [self.environment.get_constraint_by_id(
                    constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                unconstrainted_keyframe_samples = sampler.sample(self.environment, self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], [], n=number_of_samples)
                optimized_samples_raw = [optimizer.optimize(point, point) for point in unconstrainted_keyframe_samples]
                attempts = len(optimized_samples_raw)
                samples = [sample for sample in optimized_samples_raw if sample is not None]
                matched_ids = list(self.G.nodes[node]["applied_constraints"]) 
            else:
                constraints = [self.environment.get_constraint_by_id(
                    constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        
        return attempts, samples, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.G.fit_models_on_valid_samples(
            node, get_observation_joint_vector)

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.G.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.G.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher(
            '/lfd/applied_constraints', AppliedConstraints, queue_size=10)
        rospy.sleep(5)

        for i in range(len(self.G.get_keyframe_sequence()) - 1):
            cur_node = self.G.get_keyframe_sequence()[i]
            constraints = self.G.nodes[cur_node]["applied_constraints"]
            rospy.loginfo("Keyframe: {}; Constraints: {}".format(
                cur_node, constraints))
            rospy.loginfo("")

        for i in range(len(self.G.get_keyframe_sequence()) - 1):

            # Grab nodes, samples, and joints
            cur_node = self.G.get_keyframe_sequence()[i]
            next_node = self.G.get_keyframe_sequence()[i + 1]
            cur_sample = self.G.nodes[cur_node]["samples"][0]
            next_sample = self.G.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            # Build and publish node data
            time_msg = NodeTime()
            time_msg.cur_node = int(cur_node)
            time_msg.next_node = int(next_node)
            time_msg.timestamp = rospy.Time.now()
            time_pub.publish(time_msg)

            constraints = self.G.nodes[cur_node]["applied_constraints"]
            constraints_msg = AppliedConstraints()
            constraints_msg.constraints = constraints
            constraint_pub.publish(constraints_msg)
            rospy.loginfo(
                "LFD: Moving to a new point from keyframe {}".format(cur_node))
            # Execute movement using MoveIt!
            rospy.sleep(1)
            self.robot_interface.move_to_joint_targets(
                [cur_joints, next_joints])

    def generate_representation(self):
        keyframe_data = {}
        keyframe_data["point_array"] = []
        for node in self.G.get_keyframe_sequence():
            data = {}
            data["applied_constraints"] = self.G.nodes[node]["applied_constraints"]
            robot_data = {}
            robot_data["position"] = list(
                self.G.nodes[node]["samples"][random.randint(0, 5)].data["robot"]["position"])
            robot_data["orientation"] = list(
                self.G.nodes[node]["samples"][random.randint(0, 5)].data["robot"]["orientation"])
            data["robot"] = robot_data
            data["keyframe_id"] = node
            keyframe_data["point_array"].append(data)
        return keyframe_data

    def update_applied_constraints(self, applied_constraints_update):
        for node, data in applied_constraints_update.items():
            self.G.nodes[node]["applied_constraints"] = data["applied_constraints"]

    def update_constraints(self, constraint_config_update):
        new_constraints = ConstraintFactory(constraint_config_update).generate_constraints()
        current_constraints = self.environment.constraints
        for curr_idx, curr in enumerate(self.environment.constraints):
            for new_idx, new in enumerate(new_constraints):
                if new.id == curr.id:
                    self.environment.constraints[curr_idx] = new
                    new_constraints.pop(new_idx)
                    break
        constraints = current_constraints + new_constraints
        self.environment.constraints = constraints
        print("Current Environment Constraints: {}".format(self.environment.constraints))

    def serialize_out(self, path):
        data = {}
        data['config'] = self.configs
        data['labeled_demonstrations'] = [[obsv.data for obsv in demo.labeled_observations]
                                          for demo in self.G.graph['labeled_demonstrations']]
        data['intermediate_trajectories'] = {key: [[o.data for o in segment] for segment in group]
                                             for key, group in self.G.graph['intermediate_trajectories'].items()}
        data['keyframes'] = {}
        for cur_node in self.G.get_keyframe_sequence():
            data['keyframes'][cur_node] = {}
            data['keyframes'][cur_node]['applied_constraints'] = self.G.nodes[cur_node]["applied_constraints"]
            data['keyframes'][cur_node]['observations'] = [
                obsv.data for obsv in self.G.nodes[cur_node]["observations"]]
            data['keyframes'][cur_node]['keyframe_type'] = self.G.nodes[cur_node]["keyframe_type"]
        export_to_json(path, data)
        
class CC_LFD():

    def __init__(self, configs, model_settings, robot_interface):
        self.configs = configs
        self.settings = model_settings
        self.cull_overconstrained = self.settings.get(
            "cull_overconstrained", True)
        self.robot_interface = robot_interface

    def build_environment(self):
        robots = RobotFactory(self.configs['robots']).generate_robots()
        items = ItemFactory(self.configs['items']).generate_items()
        triggers = TriggerFactory(self.configs['triggers']).generate_triggers()
        constraints = ConstraintFactory(
            self.configs['constraints']).generate_constraints()
        optimizers = []
        # We only have just the one robot...for now.......
        self.environment = Environment(
            items=items, robot=robots[0], constraints=constraints, optimizers=optimizers, triggers=triggers)

    def build_keyframe_graph(self, labeled_demonstrations, bandwidth):
        self.G = KeyframeGraph()
        self.G.graph['labeled_demonstrations'] = labeled_demonstrations
        self.G.graph['intermediate_trajectories'] = IntermediateTrajectories(
        ).get_trajectories(labeled_demonstrations)
        keyframe_clustering = KeyframeClustering()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = keyframe_clustering.get_clusters(labeled_demonstrations)
        for cluster_id in clusters.keys():
            self.G.add_node(cluster_id)
            self.G.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.G.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
            self.G.nodes[cluster_id]["applied_constraints"] = clusters[cluster_id]["applied_constraints"]
            self.G.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        nx.add_path(self.G, self.G.nodes())
        self.G.fit_models(get_observation_joint_vector)
        self.G.identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples, automated_culling_threshold=False, culling_threshold=5):
        culling_threshold = self.settings.get("culling_threshold", culling_threshold)
        
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.G.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, generated_samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            rospy.loginfo(
                "Initial sampling: %s valid of %s attempts", len(generated_samples), attempts)
            if len(generated_samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    generated_samples), number_of_samples)
            if len(generated_samples) == 0:
                if self.cull_overconstrained:
                    rospy.logwarn("No valid samples found so culling keyframe from the model (this is permanent).")
                    self.G.cull_node(node)
                else:
                    rospy.logwarn("No valid samples found. Original keyframe will be kept but will likely be constraint non-compliant.")
                continue
            self.G.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in generated_samples]

            self._refit_node_model(node)
            attempts, generated_samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe: %s valid of %s attempts", len(generated_samples), attempts)
            if len(generated_samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    generated_samples), number_of_samples)
            if len(generated_samples) == 0:
                if self.cull_overconstrained:
                    rospy.logwarn("No valid samples found when refitting model so culling keyframe from the model (this is permanent).")
                    self.G.cull_node(node)
                else:
                    rospy.logwarn("No valid samples found from refitted model. Original keyframe will be kept but will likely be constraint non-compliant.")
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, generated_samples, prior_sample)
            self.G.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.G, automated_culling_threshold, culling_threshold):
            self.G.cull_node(node)

    def _generate_samples(self, node, sampler, number_of_samples):

        if self.G.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            if len(samples) == 0:
                # Some constraints couldn't be sampled successfully, so using best available samples.
                diff = list(set(self.G.nodes[node]["applied_constraints"]).difference(
                    set(matched_ids)))
                if len(matched_ids) > 0:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    constraints = [self.environment.get_constraint_by_id(
                        constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                    attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                    self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                else:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempts, samples, matched_ids, constraints

    def _refit_node_model(self, node):
        # refit models
        self.G.fit_models_on_valid_samples(
            node, get_observation_joint_vector)

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.G.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.G.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher(
            '/lfd/applied_constraints', AppliedConstraints, queue_size=10)
        rospy.sleep(5)

        for i in range(len(self.G.get_keyframe_sequence()) - 1):
            cur_node = self.G.get_keyframe_sequence()[i]
            constraints = self.G.nodes[cur_node]["applied_constraints"]
            rospy.loginfo("Keyframe: {}; Constraints: {}".format(
                cur_node, constraints))
            rospy.loginfo("")

        for i in range(len(self.G.get_keyframe_sequence()) - 1):

            # Grab nodes, samples, and joints
            cur_node = self.G.get_keyframe_sequence()[i]
            next_node = self.G.get_keyframe_sequence()[i + 1]
            cur_sample = self.G.nodes[cur_node]["samples"][0]
            next_sample = self.G.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            # Build and publish node data
            time_msg = NodeTime()
            time_msg.cur_node = int(cur_node)
            time_msg.next_node = int(next_node)
            time_msg.timestamp = rospy.Time.now()
            time_pub.publish(time_msg)

            constraints = self.G.nodes[cur_node]["applied_constraints"]
            constraints_msg = AppliedConstraints()
            constraints_msg.constraints = constraints
            constraint_pub.publish(constraints_msg)
            rospy.loginfo(
                "LFD: Moving to a new point from keyframe {}".format(cur_node))
            # Execute movement using MoveIt!
            rospy.sleep(1)
            self.robot_interface.move_to_joint_targets(
                [cur_joints, next_joints])

    def generate_representation(self):
        keyframe_data = {}
        keyframe_data["point_array"] = []
        for node in self.G.get_keyframe_sequence():
            data = {}
            data["applied_constraints"] = self.G.nodes[node]["applied_constraints"]
            robot_data = {}
            robot_data["position"] = list(
                self.G.nodes[node]["samples"][random.randint(0, 5)].data["robot"]["position"])
            robot_data["orientation"] = list(
                self.G.nodes[node]["samples"][random.randint(0, 5)].data["robot"]["orientation"])
            data["robot"] = robot_data
            data["keyframe_id"] = node
            keyframe_data["point_array"].append(data)
        return keyframe_data

    def update_applied_constraints(self, applied_constraints_update):
        for node, data in applied_constraints_update.items():
            self.G.nodes[node]["applied_constraints"] = data["applied_constraints"]

    def update_constraints(self, constraint_config_update):
        new_constraints = ConstraintFactory(constraint_config_update).generate_constraints()
        current_constraints = self.environment.constraints
        for curr_idx, curr in enumerate(self.environment.constraints):
            for new_idx, new in enumerate(new_constraints):
                if new.id == curr.id:
                    self.environment.constraints[curr_idx] = new
                    new_constraints.pop(new_idx)
                    break
        constraints = current_constraints + new_constraints
        self.environment.constraints = constraints
        print("Current Environment Constraints: {}".format(self.environment.constraints))

    def serialize_out(self, path):
        data = {}
        data['config'] = self.configs
        data['labeled_demonstrations'] = [[obsv.data for obsv in demo.labeled_observations]
                                          for demo in self.G.graph['labeled_demonstrations']]
        data['intermediate_trajectories'] = {key: [[o.data for o in segment] for segment in group]
                                             for key, group in self.G.graph['intermediate_trajectories'].items()}
        data['keyframes'] = {}
        for cur_node in self.G.get_keyframe_sequence():
            data['keyframes'][cur_node] = {}
            data['keyframes'][cur_node]['applied_constraints'] = self.G.nodes[cur_node]["applied_constraints"]
            data['keyframes'][cur_node]['observations'] = [
                obsv.data for obsv in self.G.nodes[cur_node]["observations"]]
            data['keyframes'][cur_node]['keyframe_type'] = self.G.nodes[cur_node]["keyframe_type"]
        export_to_json(path, data)


class LFD():

    def __init__(self, configs, robot_interface):
        self.configs = configs
        self.robot_interface = robot_interface

    def build_environment(self):
        robots = RobotFactory(self.configs['robots']).generate_robots()
        items = ItemFactory(self.configs['items']).generate_items()
        triggers = TriggerFactory(self.configs['triggers']).generate_triggers()
        constraints = ConstraintFactory(
            self.configs['constraints']).generate_constraints()
        # We only have just the one robot...for now.......
        self.environment = Environment(
            items=items, robot=robots[0], constraints=constraints, triggers=triggers)

    def build_keyframe_graph(self, demonstrations, bandwidth):
        self.G = KeyframeGraph()
        keyframe_clustering = KeyframeClustering()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = keyframe_clustering.get_clusters(demonstrations)
        for cluster_id in clusters.keys():
            self.G.add_node(cluster_id)
            self.G.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.G.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
            self.G.nodes[cluster_id]["applied_constraints"] = []
            self.G.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        nx.add_path(self.G, self.G.nodes())
        self.G.fit_models(get_observation_joint_vector)
        self.G.identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=-1000):
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.G.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, generated_samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            rospy.loginfo(
                "Initial sampling: %s valid of %s attempts", len(generated_samples), attempts)
            if len(generated_samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(generated_samples) == 0:
                rospy.logwarn("Couldn't fit any samples to the provided constraints")
                continue
            self.G.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe: %s valid of %s attempts", len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                rospy.logwarn("Couldn't refit the model as no samples met the provided constraints")
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, samples, prior_sample)
            self.G.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.G, automate_threshold, culling_threshold):
            self.G.cull_node(node)

    def _generate_samples(self, node, sampler, number_of_samples):

        if self.G.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            if len(samples) == 0:
                # Some constraints couldn't be sampled successfully, so using best available samples.
                diff = list(set(self.G.nodes[node]["applied_constraints"]).difference(
                    set(matched_ids)))
                if len(matched_ids) > 0:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    constraints = [self.environment.get_constraint_by_id(
                        constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
                    attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                    self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                else:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.G.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempts, samples, matched_ids, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.G.fit_models_on_valid_samples(
            node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.sample(self.environment,
                                                               self.G.nodes[node]["model"], self.G.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.G.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.G.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        # time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher(
            '/lfd/applied_constraints', AppliedConstraints, queue_size=10)
        rospy.sleep(5)
        for i in range(len(self.G.get_keyframe_sequence()) - 1):
            cur_node = self.G.get_keyframe_sequence()[i]
            constraints = self.G.nodes[cur_node]["applied_constraints"]
            rospy.loginfo("Keyframe: {}; Constraints: {}".format(
                cur_node, constraints))
            rospy.loginfo("")

        for i in range(len(self.G.get_keyframe_sequence()) - 1):

            # Grab nodes, samples, and joints
            cur_node = self.G.get_keyframe_sequence()[i]
            next_node = self.G.get_keyframe_sequence()[i + 1]
            cur_sample = self.G.nodes[cur_node]["samples"][0]
            next_sample = self.G.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            # Build and publish node data
            # time_msg = NodeTime()
            # time_msg.cur_node = int(cur_node)
            # time_msg.next_node = int(next_node)
            # time_msg.timestamp = rospy.Time.now()
            # time_pub.publish(time_msg)

            constraints = self.G.nodes[cur_node]["applied_constraints"]
            constraints_msg = AppliedConstraints()
            constraints_msg.constraints = constraints
            constraint_pub.publish(constraints_msg)
            rospy.loginfo(
                "LFD: Moving to a new point from keyframe {}".format(cur_node))
            # Execute movement using MoveIt!
            self.robot_interface.move_to_joint_targets(
                [cur_joints, next_joints])

    def generate_representation(self):
        keyframe_data = {}
        for node in self.G.get_keyframe_sequence():
            data = {}
            data["applied_constraints"] = self.G.nodes[node]["applied_constraints"]
            data["observation"] = self.G.nodes[node]["samples"][0].data
            keyframe_data[node] = data
        return keyframe_data
