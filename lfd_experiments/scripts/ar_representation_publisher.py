#!/usr/bin/env python


import rospy
import json
import time
from std_msgs.msg import String


def main():
	pub = rospy.Publisher("/cairo_lfd/lfd_representation", String, queue_size=10)
	time.sleep(.5)
	json_str = None
	with open("./cubby_representation.txt", "r") as f:
		json_str = json.loads(f.read())

	msg = String()
	msg.data = json.dumps(json_str)
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node("my_publisher_node")
	main()
