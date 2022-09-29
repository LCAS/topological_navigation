#!/usr/bin/env python

import rospy
from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import Float32
import math


class compute_human_distance(object):
	def __init__(self):
		self.zed_objects_topic_name = rospy.get_param('~zed_objects_topic_name',"objects_topic")
		self.output_topic_name = rospy.get_param('~output_topic_name',"closest_human_distance")

		#subscribers
		rospy.Subscriber(self.zed_objects_topic_name,ObjectsStamped , self.objects_callback,queue_size=1)
		self.closest_human_publisher = rospy.Publisher(self.output_topic_name, Float32,queue_size=1)

		rospy.spin()

	def objects_callback(self,data):
		closest_human_distance = 999
		if len(data.objects) > 0:
			for p in data.objects:
				if p.label == "Person":
					current_distance = math.sqrt(p.position[0]*p.position[0] + p.position[1]*p.position[1])
					if current_distance < closest_human_distance:
						closest_human_distance = current_distance

		self.closest_human_publisher.publish(closest_human_distance)
		return

if __name__ == '__main__':
	rospy.init_node('compute_closest_human_distance_node', anonymous=True)
	chd = compute_human_distance()

