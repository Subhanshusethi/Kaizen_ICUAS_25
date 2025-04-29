#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from icuas25_msgs.msg import TargetInfo
import tf2_ros
import tf2_geometry_msgs


class ArucoRelayNode(Node):
    def __init__(self, num_drones=5):
        super().__init__("aruco_relay_node")

        # TF2 Buffer and Listener to transform to the world frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List to store subscriber objects
        self.aruco_subscriptions = []

        # Subscribe to all drone topics dynamically
        for i in range(1, num_drones + 1):
            topic_name = f"/cf_{i}/aruco_markers"
            sub = self.create_subscription(
                ArucoMarkers,
                topic_name,
                self.aruco_callback,
                10
            )
            self.aruco_subscriptions.append(sub)
            self.get_logger().info(f"Subscribed to {topic_name}")

        # Publisher to the target_found topic
        self.publisher_ = self.create_publisher(
            TargetInfo,
            "target_found",
            10
        )

    def aruco_callback(self, msg):
        """
        Called whenever any /cf_x/aruco_markers topic receives a new message.
        Transforms detected ArUco markers to the world frame and publishes them.
        """
        for i, marker_id in enumerate(msg.marker_ids):
            source_frame = msg.header.frame_id  # Frame in which the marker is detected

            try:
                # Get the transformation from the drone frame to the world frame
                transform = self.tf_buffer.lookup_transform(
                    "world",  # Target frame
                    source_frame,  # Source frame (drone's frame)
                    self.get_clock().now().to_msg()  # current time stamp
                )

                # Create a PointStamped message for transformation
                marker_point = PointStamped()
                marker_point.header = msg.header
                marker_point.point = Point(
                    x=msg.poses[i].position.x,
                    y=msg.poses[i].position.y,
                    z=msg.poses[i].position.z,
                )

                # Transform the point to the world frame
                transformed_point = tf2_geometry_msgs.do_transform_point(marker_point, transform)

                # Create and publish the transformed TargetInfo message
                found_msg = TargetInfo()
                found_msg.id = marker_id  # Assuming marker_id is an int or appropriate type
                found_msg.location = transformed_point.point  # Extract the Point from PointStamped
                self.publisher_.publish(found_msg)

                self.get_logger().info(f"Published marker {marker_id} in world frame.")

            except tf2_ros.LookupException:
                self.get_logger().warn(f"Transform not available for {source_frame} -> world")

        if len(msg.marker_ids) > 0:
            self.get_logger().info(f"Relayed {len(msg.marker_ids)} markers to 'target_found' topic in world frame.")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoRelayNode(num_drones=5)  # Adjust the number of drones as needed

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
