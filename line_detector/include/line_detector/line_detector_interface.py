#!/usr/bin/env python

from abc import ABCMeta, abstractmethod
from collections import namedtuple

Detections = namedtuple('Detections',
                        ['lines', 'normals', 'area', 'centers'])


class LineDetectorInterface:
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_image(self, bgr):
        pass

    def detect_lines(self, color):
        pass

    @abstractmethod
    def get_norm_ratio(self):
        pass
