import numpy as np


def kullbach_leibler_divergence(self, P_model, Q_model, n_samples=5 * 10**3):

    # uses a sampling based approach to estimate KL divergence
    # KL(p||q) = \int p(x) log(p(x) / q(x)) dx = E_p[ log(p(x) / q(x))
    X = P_model.generate_samples(n_samples)
    logP_X = P_model.score_samples(X)
    logQ_X = Q_model.score_samples(X)
    return np.average(logP_X) - np.average(logQ_X)


def model_divergence_stats(self, graph):
    prev = graph.get_keyframe_sequence()[0]
    curr = graph.successors(prev).next()
    divergences = []
    while([x for x in graph.successors(curr)] != []):
        estimated_divergence = kullbach_leibler_divergence(graph.nodes[prev]["model"], graph.nodes[curr]["model"])
        divergences.append(estimated_divergence)
        prev = curr
        curr = graph.successors(curr).next()
    return np.average(divergences), np.std(divergences)