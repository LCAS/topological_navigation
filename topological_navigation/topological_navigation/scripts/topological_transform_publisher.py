# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# ----------------------------------
# @author: Adam Binch
# @email: abinch@sagarobotics.com
# @date: Fri Feb 26 10:37:50 2021
# ----------------------------------

import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped

# this ensures that all the poses and translates 
# are float-type and not int-type as there is an 
# assertion in ros2 messages (vector3, pose etc.) 
# for float-type [x,y,z,w] keys.
class CustomSafeLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        mapping = super().construct_mapping(node, deep=deep)

        # this can be extended to test the validity of the tmap2 
        # as well at load time (or add missing keys)
        for key in ['x', 'y', 'z', 'w']:
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])
        
        return mapping


class TopologicalTransformPublisher(Node):

    def __init__(self):
        super().__init__("topological_transform_publisher")
        self.tf_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/topological_map_2', self.map_callback, qos)
        self.get_logger().info("Transform Publisher waiting for the Topological Map...")

    def map_callback(self, msg):
        # tmap = json.loads(msg.data)
        tmap = yaml.load( msg.data, Loader=CustomSafeLoader )
        self.get_logger().info("Transform Publisher received the Topological Map")
        transformation = tmap["transformation"]
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = transformation["parent"]
        msg.child_frame_id = transformation["child"]
        msg.transform.translation.x = transformation["translation"]["x"]
        msg.transform.translation.y = transformation["translation"]["y"]
        msg.transform.translation.z = transformation["translation"]["z"]
        msg.transform.rotation.x = transformation["rotation"]["x"]
        msg.transform.rotation.y = transformation["rotation"]["y"]
        msg.transform.rotation.z = transformation["rotation"]["z"]
        msg.transform.rotation.w = transformation["rotation"]["w"]
        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)

    TTP = TopologicalTransformPublisher()
    rclpy.spin(TTP)

    TTP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
