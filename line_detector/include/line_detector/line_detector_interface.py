from abc import ABCMeta, abstractmethod


class LineDetectorInterface():
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_image(self, bgr):
        pass

    def detect_lines(self, color):
        pass