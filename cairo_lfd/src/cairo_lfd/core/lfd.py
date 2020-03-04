
import rospy


from cairo_lfd_msgs.msg import NodeTime
from cairo_lfd.core.environment import Environment
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.data.conversion import SawyerSampleConversion
from cairo_lfd.data.vectorization import get_observation_joint_vector
from cairo_lfd.constraints.acc_assignment import assign_autoconstraints, AutoconstraintFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.graphing import ObservationClusterer, KeyframeGraph
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
        items = ItemFactory(self.configs).generate_items()
        constraints = ConstraintFactory(self.configs).generate_constraints()
        # We only have just the one robot...for now.......
        self.environment = Environment(items=items['items'], robot=items['robots']
                                       [0], constraints=constraints, triggers=None)

    def build_keyframe_graph(self, demonstrations, bandwidth, vectorizor=None):
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
            self.graph.nodes[cluster_id]["autoconstraints"] = {}
            # Used to track changes in the autoconstraint assignment according to segmentation and proximity style constraint assignment.
            self.graph.nodes[cluster_id]["autoconstraint_transitions"] = []
            self.graph.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        self.graph.add_path(self.graph.nodes())
        if vectorizor is not None:
            self.graph.fit_models(vecotorizor)
        else:
            self.graph.fit_models(get_observation_joint_vector)
        self.graph.identify_primal_observations(get_observation_joint_vector)

    def generate_autoconstraints(self, demonstrations):
        rospy.loginfo("Building autoconstraints.")
        # Encapsulates all segmentation, heuristic modeling:
        autoconstraint_builders = AutoconstraintFactory(
            self.configs).generate_autoconstraint_builders(demonstrations)
        # Assigns built autosconstraints to keyframes nodes.
        assign_autoconstraints(self.graph, autoconstraint_builders)
        self._apply_autoconstraint_transitions()

    def _apply_autoconstraint_transitions(self):
        prev_set = set()
        for node in self.graph.get_keyframe_sequence():
            curr_set = set(self.graph.nodes[node]
                           ["autoconstraint_transitions"])
            diff = prev_set.symmetric_difference(curr_set)
            if len(diff) > 0:
                self.graph.nodes[node]["keyframe_type"] = "constraint_transition"
            prev_set = curr_set

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=-1000):
        number_of_samples = self.settings.get(
            "number_of_samples", number_of_samples)
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.graph.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, validated_set, autoconstraint_attempts, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            if autoconstraint_attempts == 10:
                rospy.logwarn(
                    "Not able to sample enough points for autoconstrained keyframe {}.".format(node))
                self.graph.cull_node(node)
                continue
            rospy.loginfo("Validated autoconstraints: {}".format(
                list(validated_set)))
            rospy.loginfo("Keyframe %d: %s valid of %s attempts",
                          node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            self.graph.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, samples, prior_sample)
            self.graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.graph, automate_threshold, culling_threshold):
            self.graph.cull_node(node)

    def _generate_samples(self, node, keyframe_sampler, number_of_samples, min_samples=5, constraint_attempts=10):
        validated_set = set()
        attempted_count = 0
        autoconstraint_sampler = AutoconstraintSampler(
            self.graph.nodes[node]["autoconstraints"])
        valid_samples = []
        constraints = []
        autoconstraint_attempts = 0

        while autoconstraint_sampler.validate(validated_set) is False or len(samples) < min_samples:
            if autoconstraint_attempts == constraint_attempts:
                break
            autoconstraint_attempts += 1
            constraints = autoconstraint_sampler.sample(validated_set)
            attempted_count, samples, validated_set = keyframe_sampler.sample(self.environment,
                                                                              self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            valid_samples.extend(samples)
        return attempted_count, valid_samples, validated_set, autoconstraint_attempts, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.graph.fit_models_on_valid_samples(
            node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.sample(self.environment,
                                                               self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreSampleRanker()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.graph.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.graph.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """
        joint_config_array = []
        for node in self.graph.get_keyframe_sequence():
            sample = self.graph.nodes[node]["samples"][0]
            joints = sample.get_joint_angle()
            joint_config_array.append(joints)

        self.robot_interface.move_to_joint_targets(joint_config_array)

    def generate_representation(self):
        # TODO
        pass

    def serialize_out(self):
        # TODO
        json_data = {}

    def serialize_in(self):
        # TODO
        pass


class CC_LFD():

    def __init__(self, configs, model_settings, robot_interface):
        self.configs = configs
        self.settings = model_settings
        self.cull_overconstrained = self.settings.get(
            "cull_overconstrained", True)
        self.robot_interface = robot_interface

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
            self.graph.nodes[cluster_id]["applied_constraints"] = clusters[cluster_id]["applied_constraints"]
            self.graph.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        self.graph.add_path(self.graph.nodes())
        self.graph.fit_models(get_observation_joint_vector)
        self.graph.identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=12):
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.graph.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            rospy.loginfo(
                "Initial sampling: %s valid of %s attempts", len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0 and self.cull_overconstrained:
                self.graph.cull_node(node)
                continue
            self.graph.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe: %s valid of %s attempts", len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, samples, prior_sample)
            self.graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.graph, automate_threshold, culling_threshold):
            self.graph.cull_node(node)

    def _generate_samples(self, node, sampler, number_of_samples):

        if self.graph.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            if len(samples) == 0:
                # Some constraints couldn't be sampled successfully, so using best available samples.
                diff = list(set(self.graph.nodes[node]["applied_constraints"]).difference(
                    set(matched_ids)))
                if len(matched_ids) > 0:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    constraints = [self.environment.get_constraint_by_id(
                        constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
                    attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                    self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                else:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempts, samples, matched_ids, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.graph.fit_models_on_valid_samples(
            node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.sample(self.environment,
                                                               self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceRanking()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.graph.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.graph.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher(
            '/lfd/applied_constraints', AppliedConstraints, queue_size=10)
        rospy.sleep(5)

        for i in range(len(self.graph.get_keyframe_sequence()) - 1):
            cur_node = self.graph.get_keyframe_sequence()[i]
            constraints = self.graph.nodes[cur_node]["applied_constraints"]
            rospy.loginfo("Keyframe: {}; Constraints: {}".format(
                cur_node, constraints))
            rospy.loginfo("")

        for i in range(len(self.graph.get_keyframe_sequence()) - 1):
            
            # Grab nodes, samples, and joints
            cur_node = self.graph.get_keyframe_sequence()[i]
            next_node = self.graph.get_keyframe_sequence()[i + 1]
            cur_sample = self.graph.nodes[cur_node]["samples"][0]
            next_sample = self.graph.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            Build and publish node data
            time_msg = NodeTime()
            time_msg.cur_node = int(cur_node)
            time_msg.next_node = int(next_node)
            time_msg.timestamp = rospy.Time.now()
            time_pub.publish(time_msg)

            constraints = self.graph.nodes[cur_node]["applied_constraints"]
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
        for node in self.graph.get_keyframe_sequence():
            data = {}
            data["applied_constraints"] = self.graph.nodes[node]["applied_constraints"]
            robot_data = {}
            robot_data["position"] = list(
                self.graph.nodes[node]["samples"][0].data["robot"]["position"])
            robot_data["orientation"] = list(
                self.graph.nodes[node]["samples"][0].data["robot"]["orientation"])
            data["robot"] = robot_data
            data["keyframe_id"] = node
            keyframe_data["point_array"].append(data)
        return keyframe_data

    def model_update(self, update_data):
        for node, data in update_data.items():
            self.graph.nodes[node]["applied_constraints"] = data["applied_constraints"]


class LFD():

    def __init__(self, configs, robot_interface):
        self.configs = configs
        self.robot_interface = robot_interface

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
            self.graph.nodes[cluster_id]["model"] = KDEModel(
                kernel='gaussian', bandwidth=bandwidth)
        self.graph.add_path(self.graph.nodes())
        self.graph.fit_models(get_observation_joint_vector)
        self.graph.identify_primal_observations(get_observation_joint_vector)

    def sample_keyframes(self, number_of_samples, automate_threshold=False, culling_threshold=-1000):
        sample_to_obsv_converter = SawyerSampleConversion(self.robot_interface)

        keyframe_sampler = KeyframeModelSampler(
            sample_to_obsv_converter, self.robot_interface)

        prior_sample = None
        for node in self.graph.get_keyframe_sequence():
            rospy.loginfo("")
            rospy.loginfo("KEYFRAME: {}".format(node))
            attempts, samples, matched_ids, constraints = self._generate_samples(
                node, keyframe_sampler, number_of_samples)
            rospy.loginfo(
                "Initial sampling: %s valid of %s attempts", len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            self.graph.nodes[node]["samples"] = [
                sample_to_obsv_converter.convert(sample, run_fk=True) for sample in samples]
            attempts, samples, matched_ids = self._refit_node_model(
                node, keyframe_sampler, constraints, number_of_samples)
            rospy.loginfo(
                "Refitted keyframe: %s valid of %s attempts", len(samples), attempts)
            if len(samples) < number_of_samples:
                rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(
                    samples), number_of_samples)
            if len(samples) == 0:
                self.graph.cull_node(node)
                continue
            ranked_samples = self._rank_node_valid_samples(
                node, samples, prior_sample)
            self.graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(
                sample, run_fk=True) for sample in ranked_samples]
            prior_sample = ranked_samples[0]

        # Cull candidate keyframes.
        for node in get_culling_candidates(self.graph, automate_threshold, culling_threshold):
            self.graph.cull_node(node)

    def _generate_samples(self, node, sampler, number_of_samples):

        if self.graph.nodes[node]["keyframe_type"] == "constraint_transition":
            rospy.loginfo("Sampling from a constraint transition keyframe.")
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
            if len(samples) == 0:
                # Some constraints couldn't be sampled successfully, so using best available samples.
                diff = list(set(self.graph.nodes[node]["applied_constraints"]).difference(
                    set(matched_ids)))
                if len(matched_ids) > 0:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so attempting to find valid samples with constraints {}.".format(diff, matched_ids))
                    constraints = [self.environment.get_constraint_by_id(
                        constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
                    attempts, samples, matched_ids = sampler.sample(self.environment,
                                                                    self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
                else:
                    rospy.logwarn(
                        "Constraints {} couldn't be met so. Cannot meet any constraints.".format(diff))
        else:
            constraints = [self.environment.get_constraint_by_id(
                constraint_id) for constraint_id in self.graph.nodes[node]["applied_constraints"]]
            attempts, samples, matched_ids = sampler.sample(self.environment,
                                                            self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempts, samples, matched_ids, constraints

    def _refit_node_model(self, node, sampler, constraints, number_of_samples):
        # refit models
        self.graph.fit_models_on_valid_samples(
            node, get_observation_joint_vector)
        attempted_count, samples, matched_ids = sampler.sample(self.environment,
                                                               self.graph.nodes[node]["model"], self.graph.nodes[node]["primal_observation"], constraints, n=number_of_samples)
        return attempted_count, samples, matched_ids

    def _rank_node_valid_samples(self, node, samples, prior_sample=None):
        model_score_ranker = ModelScoreRanking()
        configuration_ranker = ConfigurationSpaceSampleRanker()
        # Order sampled points based on their intra-model log-likelihood if no prior sample if the first keyframe
        if prior_sample is None:
            ranked_samples = model_score_ranker.rank(
                self.graph.nodes[node]["model"], samples)
        # else use the closest
        else:
            ranked_samples = configuration_ranker.rank(
                self.graph.nodes[node]["model"], samples, prior_sample)
        return ranked_samples

    def perform_skill(self):
        """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """

        # Create publisher for node information
        # time_pub = rospy.Publisher('/lfd/node_time', NodeTime, queue_size=10)
        constraint_pub = rospy.Publisher(
            '/lfd/applied_constraints', AppliedConstraints, queue_size=10)
        rospy.sleep(5)
        for i in range(len(self.graph.get_keyframe_sequence()) - 1):
            cur_node = self.graph.get_keyframe_sequence()[i]
            constraints = self.graph.nodes[cur_node]["applied_constraints"]
            rospy.loginfo("Keyframe: {}; Constraints: {}".format(
                cur_node, constraints))
            rospy.loginfo("")

        for i in range(len(self.graph.get_keyframe_sequence()) - 1):

            # Grab nodes, samples, and joints
            cur_node = self.graph.get_keyframe_sequence()[i]
            next_node = self.graph.get_keyframe_sequence()[i + 1]
            cur_sample = self.graph.nodes[cur_node]["samples"][0]
            next_sample = self.graph.nodes[next_node]["samples"][0]
            cur_joints = cur_sample.get_joint_angle()
            next_joints = next_sample.get_joint_angle()

            # Build and publish node data
            # time_msg = NodeTime()
            # time_msg.cur_node = int(cur_node)
            # time_msg.next_node = int(next_node)
            # time_msg.timestamp = rospy.Time.now()
            # time_pub.publish(time_msg)

            constraints = self.graph.nodes[cur_node]["applied_constraints"]
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
        for node in self.graph.get_keyframe_sequence():
            data = {}
            data["applied_constraints"] = self.graph.nodes[node]["applied_constraints"]
            data["observation"] = self.graph.nodes[node]["samples"][0].data
            keyframe_data[node] = data
        return keyframe_data
