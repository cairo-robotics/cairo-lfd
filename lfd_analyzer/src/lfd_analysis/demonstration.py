class DemonstrationConstraintAnalyzer():

    def __init__(self, environment):
        self.environment = environment

    def transition_point_identifier(self, demonstration):
        prev = []
        for observation in demonstration.observations:
            curr = observation.get_applied_constraint_data()
            if prev != curr:
                observation.data["constraint_transition"] = True
            prev = curr

    def applied_constraint_evaluator(self, demonstration):
        prev = []
        for observation in demonstration.observations:
            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered)-set(prev))
            evaluated = self.evaluator(constraint_ids=prev, observation=observation)
            applied = list(set(evaluated).union(set(new)))
            prev = applied
            observation.data["applied_constraints"] = applied

    def evaluator(self, constraint_ids, observation):

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