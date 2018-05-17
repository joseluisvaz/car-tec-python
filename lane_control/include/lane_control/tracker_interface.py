from abc import ABCMeta, abstractmethod

class TrackerInterface(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_segments(self, segment_list):
        pass
