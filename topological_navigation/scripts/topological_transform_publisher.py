#!/usr/bin/env python
"""
Created on Fri Feb 26 10:37:50 2021

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
import rospy, json
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from rospy_message_converter import message_converter
import tf2_ros


class TopologicalTransformPublisher(object):
    """This class defines a tf publisher that publishes the transformation
    between parent and child frames of a topological_map_2.
    """
    def __init__(self):

        self.rec_map=False

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber('topological_map_2', String, self.map_callback)

        rospy.loginfo("Waiting for Topological map ...")
        while not self.rec_map :
            rospy.sleep(rospy.Duration.from_sec(0.1))


    def map_callback(self, msg):

        self.tmap = json.loads(msg.data)
        self.rec_map=True

        rospy.loginfo("Topological map received")

        self.publish_transform()


    def publish_transform(self):

        transformation = self.tmap["transformation"]

        trans = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Vector3", transformation["translation"])
        rot = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Quaternion", transformation["rotation"])

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = transformation["parent"]
        msg.child_frame_id = transformation["child"]
        msg.transform.translation = trans
        msg.transform.rotation = rot

        self.broadcaster.sendTransform(msg)

#########################################################################################################


#########################################################################################################
if __name__ == '__main__':

    rospy.init_node("topological_transform_publisher")

    TopologicalTransformPublisher()
    rospy.spin()
#########################################################################################################