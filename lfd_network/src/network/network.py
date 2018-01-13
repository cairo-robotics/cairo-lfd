class KeyframeNode:

    def __init__(self, GMM, samples, keyframe_type="standard"):
        self.gmm = GMM
        self.simulated_samples = samples
        self.keyframe_type = keyframe_type
        self.valid_points

    def regenerate_samples(self):
        self.simulated_samples = self.gmm.generate_samples()

    def evaluate_points(self):
        pass


class WaypointGenerator:

    def generate_way_points(self, keyframe1, keyframe2):
        pass



