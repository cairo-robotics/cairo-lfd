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


class DataGrouper():

    def __init__(self, demonstrations, constraint_transitions):
        self.demonstrations = demonstrations
        self.constraint_transitions

    def observation_grouper(self, observations, constraint_transitions, window):
        for group

    def keyframe_grouper(self, )
        pass

    def _retrieve_window_points(self, observations, central_idx, window_size):
        group = []
        for spread in reversed(range(window_size+1)):
            print(spread)
            if 0 <= central_idx-spread < len(observations) and 0 <= central_idx+spread < len(observations):
                group.extend(observations[central_idx-spread:central_idx+spread+1])
                break
        return group



if __name__ == "__main__":
    pass

