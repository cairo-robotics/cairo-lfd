

def map_constraints(demonstration, evaluator):
    prev = []
    for observation in demonstration.observations:
        triggered = observation.get_triggered_constraint_data()
        new = list(set(triggered)-set(prev))
        evaluated = evaluator(constraint_ids=prev, observation=observation)
        applied = list(set(evaluated).union(set(new)))
        prev = applied
        observation.data["applied_constraints"] = applied


def evaluator(environment, constraint_ids, observation):
    if constraint_ids is not []:
        valid_constraints = []
        for constraint_id in constraint_ids:
            constraint = environment.get_constraint_by_id(constraint_id)
            result = constraint.evaluate(environment, observation)
            if result is 1:
                valid_constraints.append(constraint_id)
        return valid_constraints
    else:
        return []
