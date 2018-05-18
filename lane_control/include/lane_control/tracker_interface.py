from abc import ABCMeta, abstractmethod


class TrackerInterface(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def track_segments(self, segment_list):
        pass
