#!/usr/bin/env python3
"""ROS2 node for publishing Meta Quest hand transforms to TF.

This node uses MetaQuestReader to get hand transforms and publishes
them to ROS2 TF in the meta_world frame. The coordinate system conversion
from OpenXR to ROS is handled by tf2 via a static transform publisher.

This follows the correct ROS2 tf2 approach:
1. Publish Meta Quest data in its own frame (meta_world)
2. Use static_transform_publisher to link meta_world to ROS world frame
3. Let tf2 handle all coordinate system transformations automatically

The node also handles homing/relative tracking functionality for TF transforms
by setting a home pose with Button B and resetting with Button A. Pose topics
are always published in absolute coordinates.
"""

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from meta_quest_teleop.reader import MetaQuestReader


class MetaQuestTFPublisher(Node):
    """Publishes Meta Quest hand transforms to TF in meta_world frame.

    Handles grip, pointer, and model transforms per hand. Button B sets home pose
    (relative tracking), Button A resets to absolute tracking.
    """

    def __init__(self) -> None:
        """Initialize node, connect to Meta Quest, set up publishers (50 Hz)."""
        super().__init__("meta_quest_tf_publisher")

        # Declare parameters with proper typed defaults
        ip_address = None
        port = 5555
        update_rate = 50.0

        # Convert empty string to None for optional IP address

        # Publish Meta Quest data in its own frame - tf2 handles conversion
        self.world_frame = "map"
        self.meta_frame = "meta_world"

        # Define transform types and frame names
        self.transform_types = ["grip", "pointer", "model"]
        self.left_hand_frames = {
            "grip": "left_hand_grip",
            "pointer": "left_hand_pointer",
            "model": "left_hand_model",
        }
        self.right_hand_frames = {
            "grip": "right_hand_grip",
            "pointer": "right_hand_pointer",
            "model": "right_hand_model",
        }

        # Home pose management for TF only (for all transform types)
        self.home_poses_left = {
            transform_type: None for transform_type in self.transform_types
        }
        self.home_poses_right = {
            transform_type: None for transform_type in self.transform_types
        }
        self.use_relative_tracking = True

        # Initialize Meta Quest Reader
        self.get_logger().info(
            f"Connecting to Meta Quest (IP: {ip_address}, Port: {port})..."
        )
        self.reader = MetaQuestReader(ip_address=ip_address, port=port)
        self.get_logger().info("Connected to Meta Quest!")

        # Register button callbacks
        self.reader.on("button_b_pressed", self._on_button_b_pressed)
        self.reader.on("button_a_pressed", self._on_button_a_pressed)

        # Initialize TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform from meta_world to ROS world frame (map)
        self._publish_static_transform()

        # Create publishers for ROS-converted poses (for testing/validation)
        self.pose_publishers = {}
        for hand in ["left", "right"]:
            for transform_type in self.transform_types:
                topic_name = f"/meta_quest/{hand}_{transform_type}_pose"
                self.pose_publishers[f"{hand}_{transform_type}"] = (
                    self.create_publisher(PoseStamped, topic_name, 10)
                )

        # Create publishers for velocities
        self.twist_publishers = {}
        for hand in ["left", "right"]:
            for transform_type in self.transform_types:
                topic_name = f"/meta_quest/{hand}_{transform_type}_velocity"
                self.twist_publishers[f"{hand}_{transform_type}"] = (
                    self.create_publisher(TwistStamped, topic_name, 10)
                )

        # Track previous poses and time for velocity calculation
        self.prev_poses: dict[str, np.ndarray] = {}
        self.prev_time = self.get_clock().now()

        # Create timer for publishing transforms
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.publish_topics)

        self._print_intro()

    def _print_intro(self) -> None:
        """Log initialization info (frames, transform types, controls)."""
        self.get_logger().info(f"Parent frame: {self.meta_frame}")
        self.get_logger().info(f'Transform types: {", ".join(self.transform_types)}')
        self.get_logger().info(
            f'Left hand frames: {", ".join(self.left_hand_frames.values())}'
        )
        self.get_logger().info(
            f'Right hand frames: {", ".join(self.right_hand_frames.values())}'
        )
        self.get_logger().info("")
        self.get_logger().info("✅ Static transform published: map -> meta_world")
        self.get_logger().info(
            "✅ Meta Quest transforms published in meta_world frame (OpenXR)"
        )
        self.get_logger().info(
            "✅ tf2 automatically handles coordinate system conversion"
        )
        self.get_logger().info("")
        self.get_logger().info("Press button B to set home pose for TF (zero position)")
        self.get_logger().info(
            "Press button A to reset home pose for TF (absolute tracking)"
        )

    def _publish_static_transform(self) -> None:
        """Publish static transform from map to meta_world (OpenXR->ROS)."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = self.world_frame
        static_transform.child_frame_id = self.meta_frame

        # No translation (assume origins are the same)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        # Quaternion for OpenXR to ROS coordinate system conversion
        # This rotates: X=right->forward, Y=up->left, Z=backward->up
        static_transform.transform.rotation.x = 0.5
        static_transform.transform.rotation.y = -0.5
        static_transform.transform.rotation.z = -0.5
        static_transform.transform.rotation.w = 0.5

        # Publish the static transform
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info(
            f"Published static transform: {self.world_frame} -> {self.meta_frame}"
        )

    def _on_button_b_pressed(self) -> None:
        """Set home pose for relative tracking (all hands, all transform types)."""
        # Get current transforms for both hands
        left_transform = self.reader.get_hand_controller_transform_openxr("left")
        right_transform = self.reader.get_hand_controller_transform_openxr("right")

        # Set home poses for all transform types (same transform for all types now)
        for transform_type in self.transform_types:
            if left_transform is not None:
                self.home_poses_left[transform_type] = left_transform.copy()
                self._log_home_set(
                    "left", transform_type, self.home_poses_left[transform_type]
                )

            if right_transform is not None:
                self.home_poses_right[transform_type] = right_transform.copy()
                self._log_home_set(
                    "right", transform_type, self.home_poses_right[transform_type]
                )

        self.use_relative_tracking = True
        self.get_logger().info(
            "🏠 Home pose set for TF! Tracking relative to current position."
        )

    def _on_button_a_pressed(self) -> None:
        """Reset home pose and enable absolute tracking."""
        # Reset home poses (for TF only)
        self.home_poses_left = {
            transform_type: None for transform_type in self.transform_types
        }
        self.home_poses_right = {
            transform_type: None for transform_type in self.transform_types
        }
        self.prev_poses = {}

        self.use_relative_tracking = False
        self.get_logger().info(
            "🔄 Home pose reset for TF! Tracking in absolute coordinates."
        )

    def _log_home_set(
        self, hand: str, transform_type: str, transform: np.ndarray
    ) -> None:
        """Log home pose position and rotation.

        Args:
            hand: 'left' or 'right'
            transform_type: 'grip', 'pointer', or 'model'
            transform: 4x4 transform matrix
        """
        position = transform[:3, 3]
        euler = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
        self.get_logger().info(
            f"  {hand.capitalize()} hand ({transform_type}): "
            f"pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] "
            f"rot=[{euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f}]"
        )

    def _matrix_to_pose_stamped(
        self, matrix: np.ndarray, frame_id: str
    ) -> Optional[PoseStamped]:
        """Convert 4x4 transform matrix to PoseStamped.

        Args:
            matrix: 4x4 transform matrix (ROS coordinates)
            frame_id: Frame ID for message header

        Returns:
            PoseStamped message or None if invalid
        """
        # Validate rotation matrix
        det = np.linalg.det(matrix[:3, :3])
        if abs(abs(det) - 1.0) > 0.1:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id

        # Set position
        pose.pose.position.x = float(matrix[0, 3])
        pose.pose.position.y = float(matrix[1, 3])
        pose.pose.position.z = float(matrix[2, 3])

        try:
            # Convert rotation to quaternion
            rotation = Rotation.from_matrix(matrix[:3, :3])
            quaternion = rotation.as_quat()  # [x, y, z, w]

            pose.pose.orientation.x = float(quaternion[0])
            pose.pose.orientation.y = float(quaternion[1])
            pose.pose.orientation.z = float(quaternion[2])
            pose.pose.orientation.w = float(quaternion[3])

            return pose
        except ValueError:
            return None

    def _matrix_to_transform_stamped(
        self, matrix: np.ndarray, frame_id: str, child_frame_id: str
    ) -> Optional[TransformStamped]:
        """Convert 4x4 transform matrix to TransformStamped.

        Args:
            matrix: 4x4 transform matrix (OpenXR coordinates)
            frame_id: Parent frame ID (typically 'meta_world')
            child_frame_id: Child frame ID (hand frame name)

        Returns:
            TransformStamped message or None if invalid
        """
        # Validate rotation matrix
        det = np.linalg.det(matrix[:3, :3])
        if abs(abs(det) - 1.0) > 0.1:
            self.get_logger().warning(
                f"Invalid rotation matrix for {child_frame_id} "
                f"(det={det:.3f}), skipping..."
            )
            return None

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        # Set translation
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])

        try:
            # Convert rotation to quaternion
            rotation = Rotation.from_matrix(matrix[:3, :3])
            quaternion = rotation.as_quat()  # Returns [x, y, z, w]

            transform.transform.rotation.x = float(quaternion[0])
            transform.transform.rotation.y = float(quaternion[1])
            transform.transform.rotation.z = float(quaternion[2])
            transform.transform.rotation.w = float(quaternion[3])

            return transform

        except ValueError as e:
            self.get_logger().warning(
                f"Failed to convert rotation for {child_frame_id}: {e}"
            )
            return None

    def _publish_velocity(
        self,
        hand: str,
        transform_type: str,
        current_transform: np.ndarray,
        current_time: Time,
    ) -> None:
        """Calculate and publish linear/angular velocity from pose changes.

        Args:
            hand: 'left' or 'right'
            transform_type: 'grip', 'pointer', or 'model'
            current_transform: Current 4x4 transform matrix (ROS coordinates)
            current_time: Current ROS time
        """
        key = f"{hand}_{transform_type}"

        # Need previous pose to calculate velocity
        if key not in self.prev_poses:
            self.prev_poses[key] = current_transform.copy()
            return

        # Calculate time delta
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        if dt <= 0:
            return

        prev_transform = self.prev_poses[key]

        # Calculate linear velocity (position change / time)
        linear_vel = (current_transform[:3, 3] - prev_transform[:3, 3]) / dt

        # Calculate angular velocity (rotation change / time)
        # R_diff = R_current @ R_prev^T gives the rotation from prev to current
        R_diff = current_transform[:3, :3] @ prev_transform[:3, :3].T

        try:
            # Convert rotation difference to axis-angle (rotation vector)
            # The rotation vector direction is the axis, magnitude is the angle
            angular_vel = Rotation.from_matrix(R_diff).as_rotvec() / dt
        except ValueError:
            angular_vel = np.zeros(3)

        # Create and publish TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = self.world_frame

        # Linear velocity (m/s)
        twist_msg.twist.linear.x = float(linear_vel[0])
        twist_msg.twist.linear.y = float(linear_vel[1])
        twist_msg.twist.linear.z = float(linear_vel[2])

        # Angular velocity (rad/s)
        twist_msg.twist.angular.x = float(angular_vel[0])
        twist_msg.twist.angular.y = float(angular_vel[1])
        twist_msg.twist.angular.z = float(angular_vel[2])

        self.twist_publishers[key].publish(twist_msg)

        # Update stored pose for next iteration
        self.prev_poses[key] = current_transform.copy()

    def get_transform_relative_to_home(
        self, transform: np.ndarray, home_pose: Optional[np.ndarray]
    ) -> np.ndarray:
        """Compute transform relative to home pose (T_home^-1 @ T_current).

        Args:
            transform: Current absolute 4x4 transform (OpenXR coordinates)
            home_pose: Home pose 4x4 transform or None

        Returns:
            Relative transform if enabled and home set, else absolute transform
        """
        if not self.use_relative_tracking or home_pose is None:
            return transform

        try:
            home_inv = np.linalg.inv(home_pose)
            return home_inv @ transform
        except np.linalg.LinAlgError:
            self.get_logger().warning(
                "Failed to invert home pose, using absolute transform"
            )
            return transform

    def publish_topics(self) -> None:
        """Publish transforms (TF), poses, and velocities for all hands/types.

        TF uses relative tracking if enabled, poses always use absolute coords.
        """
        # Get current time for all messages
        current_time = self.get_clock().now()

        # Publish transforms for both hands and all transform types
        # Note: The MetaQuestReader now provides a single transform per hand,
        # so we use the same transform for all transform types (grip, pointer, model)

        # Get transforms once per hand (reused for all transform types)
        right_transform_openxr = self.reader.get_hand_controller_transform_openxr(
            "right"
        )
        left_transform_openxr = self.reader.get_hand_controller_transform_openxr("left")
        right_transform_ros = self.reader.get_hand_controller_transform_ros("right")
        left_transform_ros = self.reader.get_hand_controller_transform_ros("left")

        for transform_type in self.transform_types:
            # Publish right hand transform
            if right_transform_openxr is not None:
                right_relative_transform = self.get_transform_relative_to_home(
                    right_transform_openxr, self.home_poses_right[transform_type]
                )

                # Convert to ROS2 message and publish to TF
                right_tf_msg = self._matrix_to_transform_stamped(
                    right_relative_transform,
                    self.meta_frame,
                    self.right_hand_frames[transform_type],
                )

                if right_tf_msg is not None:
                    self.tf_broadcaster.sendTransform(right_tf_msg)

            if right_transform_ros is not None:
                right_pose_msg = self._matrix_to_pose_stamped(
                    right_transform_ros, self.world_frame
                )
                if right_pose_msg is not None:
                    self.pose_publishers[f"right_{transform_type}"].publish(
                        right_pose_msg
                    )

                # Publish velocity
                self._publish_velocity(
                    "right", transform_type, right_transform_ros, current_time
                )

            # Publish left hand transform
            if left_transform_openxr is not None:
                left_relative_transform = self.get_transform_relative_to_home(
                    left_transform_openxr, self.home_poses_left[transform_type]
                )

                # Convert to ROS2 message and publish to TF
                left_tf_msg = self._matrix_to_transform_stamped(
                    left_relative_transform,
                    self.meta_frame,
                    self.left_hand_frames[transform_type],
                )

                if left_tf_msg is not None:
                    self.tf_broadcaster.sendTransform(left_tf_msg)

            if left_transform_ros is not None:
                left_pose_msg = self._matrix_to_pose_stamped(
                    left_transform_ros, self.world_frame
                )
                if left_pose_msg is not None:
                    self.pose_publishers[f"left_{transform_type}"].publish(
                        left_pose_msg
                    )

                # Publish velocity
                self._publish_velocity(
                    "left", transform_type, left_transform_ros, current_time
                )

        # Update time for next velocity calculation
        self.prev_time = current_time


def main(args: Optional[list[str]] = None) -> None:
    """Initialize ROS2, create node, and spin until interrupted.

    Args:
        args: Command-line arguments for ROS2 initialization
    """
    rclpy.init(args=args)
    node = MetaQuestTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
