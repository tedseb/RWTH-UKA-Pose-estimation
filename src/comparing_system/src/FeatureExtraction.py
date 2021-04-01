"""
This file contains code that deals with the  manual extraction of low-level features from skelletons.
At the time of writing, such features are explicitly angles. Other features might follow in the future.
"""
from abc import ABC, abstractmethod


class FeatureExtractor(ABC):
    @abstractmethod
    def __init__():
        pass

    @abstractmethod
    def extract_features():
        pass


class SpinFeatureExtractor(ABC):
    def __init__():
        raise NotImplementedError

    def extract_features():
        raise NotImplementedError
