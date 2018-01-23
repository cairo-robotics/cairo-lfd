from lfd_data.processing import DataProcessor
from lfd_data.io import DataImporter




def analyze_observation(self, analyzer_formatter, analyzer):
    return analyzer(*analyzer_formatter())
