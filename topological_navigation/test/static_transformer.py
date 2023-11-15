import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros

def main():
    rclpy.init()

    node = rclpy.create_node('static_transform_publisher')

    broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.frame_id = 'topo_map' 
    static_transform_stamped.child_frame_id = 'base_link'

    static_transform_stamped.transform.translation.x = 0.0 
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0

    static_transform_stamped.transform.rotation.x = 0.0 
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0

    # Publish the static transform
    broadcaster.sendTransform(static_transform_stamped)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()