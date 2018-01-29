


def map_constraints(trajectory, evaluator):
    prev = []
    for idx, observation in enumerate(trajectory.observations):
        triggered = observation["triggered_constraints"]
        new = list(set(triggered)-set(triggered).intersection(set(prev)))
        evaluated = evaluator(prev)
        applied = list(set(evaluated).union(set(new)))
        observation["applied_constraints"] = applied


def evaluator(constraint_ids, environment, observation):
    if constraint_ids is not []:
        for constraint_id in constraint_ids:
            constraint = environment.get_constraint(constraint_id)
            constraint.evaluate()
    else:
        return []
