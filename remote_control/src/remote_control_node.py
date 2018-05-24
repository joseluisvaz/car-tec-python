#!/usr/bin/env python

import rospy

from remote_control import remote_control_node_impl

if __name__ == "__main__":
    try:
        print("tomala papa")
        remote_control_node_impl.main()
    except rospy.ROSInterruptException:
        pass
