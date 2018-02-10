from lfd_processor.environment import Demonstration, Observation
from lfd_processor.data_io import DataImporter
from lfd_processor.alignment import DemonstrationAligner, vectorize_demonstration

class ConstraintAnalyzer():

    def __init__(self, environment):
        self.environment = environment

    def transition_point_identifier(self, observations):
        prev = []
        for observation in observations:
            curr = observation.get_applied_constraint_data()
            if prev != curr:
                observation.data["constraint_transition"] = True
            prev = curr

    def applied_constraint_evaluator(self, observations):
        prev = []
        for observation in observations:
            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered)-set(prev))
            evaluated = self._evaluator(constraint_ids=prev, observation=observation)
            applied = list(set(evaluated).union(set(new)))
            prev = applied
            observation.data["applied_constraints"] = applied

    def _evaluator(self, constraint_ids, observation):

        if constraint_ids is not []:
            valid_constraints = []
            for constraint_id in constraint_ids:
                constraint = self.environment.get_constraint_by_id(constraint_id)
                result = constraint.evaluate(self.environment, observation)
                if result is 1:
                    valid_constraints.append(constraint_id)
            return valid_constraints
        else:
            return []


class ObservationGrouper():

    def __init__(self, demonstrations = None):
        self.demonstrations = demonstrations

    def separate_transition_from_normal_data(self, observations, constraint_transitions):
        groups = []

        for idx in range(2*len(constraint_transitions)+1):
            groups.append({
                    "observations": [],
                    "type": "normal" if idx % 2 == 0 else "transition"
                })
        curr_constraints = []
        counter = 0
        print(len(groups))
        for idx, ob in enumerate(observations):
            curr_group = groups[counter]
            if ob.data["applied_constraints"] != curr_constraints:
                counter += 1
                curr_group = groups[counter]
                curr_group["observations"].extend(self._retrieve_window_points(observations, idx, window_size=3))
                curr_constraints = constraint_transitions.pop(0) if len(constraint_transitions) > 0 else []
                counter += 1
                curr_group = groups[counter]
            else:
                curr_group["observations"].append(ob)
        return groups

    def keyframe_grouper(self, raw_groups):
        pass

    def _retrieve_window_points(self, observations, central_idx, window_size=5):
        group = []
        for spread in reversed(range(window_size+1)):
            if 0 <= central_idx-spread < len(observations) and 0 <= central_idx+spread < len(observations):
                group.extend(observations[central_idx-spread:central_idx+spread+1])
                break
        return group


if __name__ == "__main__":
    importer = DataImporter()
    trajectories = importer.load_json_files('./src/lfd/lfd_processor/src/lfd_processor/*.json')

    # Convert trajectory data into Demonstrations and Observations
    demonstrations = []
    for datum in trajectories["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    aligner = DemonstrationAligner(demonstrations, vectorize_demonstration)
    aligned_demos = aligner.align()

    print "Demonstration Constraint Transitions"
    for demo in aligned_demos:
        print demo.get_applied_constraint_order()

    grouper = ObservationGrouper()
    groupings = []
    for demo in aligned_demos:
        groupings.append(grouper.separate_transition_from_normal_data(demo.aligned_observations, demo.get_applied_constraint_order()))

