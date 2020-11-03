#!/usr/bin/env python2
import os
import json
import time
import signal

import rospy
from std_msgs.msg import String

from cairo_lfd.data.io import load_json_file

class GracefulQuit:
    kill = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill = True

def main():
    rospy.init_node("constraint_update_test")
    pub = rospy.Publisher('/cairo_lfd/constraint_update', String, queue_size=10)
    fd = os.path.dirname(os.path.abspath(__file__))
    ar_constraint_data = load_json_file(fd + "/ar_constraints_test.json")
    print("Publishing: {}".format(json.dumps(ar_constraint_data)))
    g_quit = GracefulQuit()
    while not g_quit.kill:
        rospy.sleep(1)
        pub.publish(json.dumps(ar_constraint_data))

if __name__ == "__main__":
    main()
