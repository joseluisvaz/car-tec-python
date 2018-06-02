#!/usr/bin/env python

from __future__ import print_function

import rospy
from inputs import devices
from inputs import get_gamepad
from scipy.interpolate import interp1d

from std_msgs.msg import Float32MultiArray


class RemoteControlNodeImpl(object):

    def __init__(self):

        self.xbox_pad = devices.gamepads[0]

        self.steer_command = 0
        self.gas_command = 0
        self.brake_command = 0

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Float32MultiArray,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def get_input_array(self):
        events = get_gamepad()
        for e in events:
            if e.code is 'ABS_X':

                if abs(e.state) < 4000:
                    e.state = 0

                m = interp1d([-35000, 35000], [0, 255])
                self.steer_command = int(m(e.state))

            if e.code is 'ABS_Z':
                self.gas_command = e.state

            if e.code is 'BTN_SOUTH':
                if e.state is 1:
                    self.brake_command = 1
                elif e.state is 0:
                    self.brake_command = 0

        return [self.steer_command, self.gas_command, self.brake_command]


def main():
    rospy.init_node('remote_control', anonymous=True)
    rate = rospy.Rate(100)  # 100hz

    remote_control = RemoteControlNodeImpl()

    while not rospy.is_shutdown():
        message = Float32MultiArray()
        message.data = remote_control.get_input_array()
        message.data[0] = (message.data[0] - 127) * 23 / 127
        message.data[0] *= (-1)
        message.data[1] *= (100.0/255.0)

        remote_control.control_pub.publish(message)
        rate.sleep()
