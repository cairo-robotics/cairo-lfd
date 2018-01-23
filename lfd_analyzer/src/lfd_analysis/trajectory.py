from lfd_data.processing import DataProcessor
from lfd_data.io import DataImporter


class TrajectoryConstraintMapper(object):

    def __init__(self, trajectories):
        self.trajectories = trajectories

    def _analyze_observation(self, analyzer_formatter, analyzer):
        return analyzer(*analyzer_formatter())
