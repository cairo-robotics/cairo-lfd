
import rospy

from cairo_lfd.core.environment import Environment
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.data.conversion import SawyerSampleConverter
from cairo_lfd.data.vectorization import get_observation_joint_vector
from cairo_lfd.constraints.metaconstraint_assignment import MetaconstraintAssigner, MetaconstraintBuilderFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.graphing import ObservationClusterer, KeyframeGraph
from cairo_lfd.modeling.models import KDEModel
from cairo_lfd.modeling.sampling import KeyframeSampler, ModelScoreSampleRanker, ConfigurationSpaceSampleRanker, MetaconstraintSampler
from cairo_lfd.modeling.analysis import KeyframeGraphAnalyzer, ConstraintAnalyzer

from cairo_lfd_msgs.msg import KeyframeConstraints


class ACC_LFD():

    def __init__(self, configs, moveit_interface):
        self.configs = configs
        self.moveit_interface = moveit_interface

    def build_environment(self):
        items = ItemFactory(self.configs).generate_items()
        constraints = ConstraintFactory(self.configs).generate_constraints()
        # We only have just the one robot...for now.......
        self.environment = Environment(items=items['items'], robot=items['robots']
                                       [0], constraints=constraints, triggers=None)

    def build_keyframe_graph(self, demonstrations, bandwidth):
        self.graph = KeyframeGraph()
        cluster_generator = ObservationClusterer()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = cluster_generator.generate_clusters(demonstrations)
        for cluster_id in clusters.keys():
            self.graph.add_node(cluster_id)
            self.graph.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.graph.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]

            ####################################################
            # NEED TO BE ABLE TO SUPPORT CC-LFD eventually!
            # graph.nodes[cluster_id]["applied_constraints"] = [clusters[cluster_id]["applied_constraints"]]
            self.graph.nodes[cluster_id]["applied_constraints"] = []
            ####################################################
            self.graph.nodes[cluster_id]["metaconstraints"] = {}
            self.graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=bandwidth)
        self.graph.add_path(self.graph.nodes())
        self.graph.fit_models(get_observation_joint_vector)
        self.graph._identify_primal_observations(get_observation_joint_vector)

    def generate_metaconstraints(self, demonstrations):
        # Encapsulates all segmentation, heuristic modeling:
        metaconstraint_builders = MetaconstraintBuilderFactory(
            self.configs).generate_metaconstraint_builders(demonstrations)
        # Assigns built metasconstraints to keyframes nodes.
        metaconstraint_assigner = MetaconstraintAssigner(self.graph, metaconstraint_builders)
        metaconstraint_assigner.assign_metaconstraints()

    def sample_keyframes(self, number_of_samples):
        sample_to_obsv_converter = SawyerSampleConverter(self.moveit_interface)

        """ Build a ConstraintAnalyzer and KeyframeGraphAnalyzer """
        constraint_analyzer = ConstraintAnalyzer(self.environment)

        keyframe_sampler = KeyframeSampler(constraint_analyzer, sample_to_obsv_converter, self.moveit_interface)

        prior_sample = None
        for node in self.graph.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, validated_set, metaconstraint_attempts, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            if metaconstraint_attempts == 5:
                rospy.logwarn("Not able to sample enough points for metaconstrained keyframe {}.".format(node))
                self.graph.cull_node(node)
                continue
            rospy.loginfo("Validated metaconstraints: {}".format(list(validated_set)))
            rospy.loginfo("Keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            self.graph.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo("Refitted keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(node, samples, prior_sample)
            self.graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]
        self._cull_consecutive_keyframes()

    def _generate_samples(self, node, sampler, number_of_samples, min_samples=5, constraint_attempts=10):
        validated_set = set()
        attempted_count = 0
        meta_sampler = MetaconstraintSampler(self.graph.nodes[node]["metaconstraints"])
        samples = []
        constraints = []
        metaconstraint_attempts = 0

        while meta_sampler.validate(validated_set) is False or len(samples) < min_samples:
            if metaconstraint_attempts == constraint_attempts:
                break
            metaconstraint_attempts += 1
            constraints = meta_sampler.sample(validated_set)
            attempted_count, samples, validated_set = sampler.generate_n_valid_samples(
                self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, validated_set, metaconstraint_attempts, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.graph.fit_models_on_valid_samples(node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.generate_n_valid_samples(
            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreSampleRanker()
        configuration_ranker = ConfigurationSpaceSampleRanker()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(self.graph.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(self.graph.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def _cull_consecutive_keyframes(self):
        graph_analyzer = KeyframeGraphAnalyzer(self.graph, self.moveit_interface, get_observation_joint_vector)
        graph_analyzer.cull_keyframes()

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """
        joint_config_array = []
        for node in self.graph.get_keyframe_sequence():
            sample = self.graph.nodes[node]["samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

        self.moveit_interface.move_to_joint_targets(joint_config_array)

    def serialize_out(self):
        json_data = {}
        for node in self.graph.get_keyframe_sequence():
            sample = self.graph.nodes[node]["samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

    def serialize_in(self):
        pass


class CC_LFD():

    def __init__(self, configs, moveit_interface):
        self.configs = configs
        self.moveit_interface = moveit_interface

    def build_environment(self):
        items = ItemFactory(self.configs).generate_items()
        constraints = ConstraintFactory(self.configs).generate_constraints()
        # We only have just the one robot...for now.......
        self.environment = Environment(items=items['items'], robot=items['robots']
                                       [0], constraints=constraints, triggers=None)

    def build_keyframe_graph(self, demonstrations, bandwidth):
        self.graph = KeyframeGraph()
        cluster_generator = ObservationClusterer()

        """
        Generate clusters using labeled observations, build the models, graphs, and attributes for each
        cluster in the KeyFrameGraph
        """
        clusters = cluster_generator.generate_clusters(demonstrations)
        for cluster_id in clusters.keys():
            self.graph.add_node(cluster_id)
            self.graph.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
            self.graph.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
            self.graph.nodes[cluster_id]["applied_constraints"] = []
            self.graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=bandwidth)
        self.graph.add_path(self.graph.nodes())
        self.graph.fit_models(get_observation_joint_vector)
        self.graph._identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples):
        sample_to_obsv_converter = SawyerSampleConverter(self.moveit_interface)

        """ Build a ConstraintAnalyzer and KeyframeGraphAnalyzer """
        constraint_analyzer = ConstraintAnalyzer(self.environment)

        keyframe_sampler = KeyframeSampler(constraint_analyzer, sample_to_obsv_converter, self.moveit_interface)

        prior_sample = None
        for node in self.graph.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            self.graph.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo("Refitted keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(node, samples, prior_sample)
            self.graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]
        self._cull_consecutive_keyframes()

    def _generate_samples(self, node, sampler, number_of_samples):

        if self.graph.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.generate_n_valid_samples(
                self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            if len(samples) == 0:
                # Some constraints couldn't be sampled successfully, so using best available samples.
                diff = list(set(self.graph.nodes[node]["applied_constraints"]).difference(set(matched_ids)))
                if len(matched_ids) > 0:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    constraints = [self.environment.get_constraint_by_id(
                        constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
                    attempts, samples, matched_ids = sampler.generate_n_valid_samples(
                        self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                else:
                    rospy.logwarn("Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.generate_n_valid_samples(
                self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempts, samples, matched_ids, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.graph.fit_models_on_valid_samples(node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.generate_n_valid_samples(
            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreSampleRanker()
        configuration_ranker = ConfigurationSpaceSampleRanker()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(self.graph.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(self.graph.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def _cull_consecutive_keyframes(self):
        graph_analyzer = KeyframeGraphAnalyzer(self.graph, self.moveit_interface, get_observation_joint_vector)
        graph_analyzer.cull_keyframes()

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """
        joint_config_array = []
        for node in self.graph.get_keyframe_sequence():
            sample = self.graph.nodes[node]["samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

        self.moveit_interface.move_to_joint_targets(joint_config_array)

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher('/lfd/applied_constraints', KeyframeConstraints)

        for i in range(len(self.graph.get_keyframe_sequence()) - 1):
            rospy.loginfo("LFD: Moving to a new point...")

            # Grab nodes, samples, and joints
            cur_node = self.graph.get_keyframe_sequence()[i]
            next_node = self.graph.get_keyframe_sequence()[i + 1]
            cur_sample = self.graph.nodes[cur_node]["samples"][0]
            next_sample = self.graph.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            # Build and publish node data
            time_msg = NodeTime()
            time_msg.cur_node = int(cur_node)
            time_msg.next_node = int(next_node)
            time_msg.timestamp = rospy.Time.now()
            time_pub.publish(time_msg)

            constraints = self.graph.nodes[cur_node]["applied_constraints"]
            constraints_msg = KeyframeConstraints()
            constraints_msg.constraints = constraints
            constraint_pub.publish(constraints_msg)

            # Execute movement using MoveIt!
            self.moveit_interface.move_to_joint_targets([cur_joints, next_joints])
