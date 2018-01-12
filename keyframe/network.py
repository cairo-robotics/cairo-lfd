class KeyframeNode:

    def __init__(self, GMM, samples, keyframe_type="standard"):
        self.gmm = GMM
        self.samples = samples
        self.keyframe_type = keyframe_type

    def regenerate_samples(self):
        self.samples = self.gmm.generate_samples()


class WaypointGenerator:

    def generate_way_points(self):
        pass



