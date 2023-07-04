#!/usr/bin/env python3
"""Example of a node publishing current Pose and suscribing to predictions
We load the teleop dataset and loop over the first test trajectory
that gets published as PoseArrays at 10Hz in "/prescyent/position"
"""
import rclpy
from geometry_msgs.msg import PoseArray
from prescyent.dataset import TeleopIcubDataset, TeleopIcubDatasetConfig
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from ros2_prescyent.utils.pose_helper import get_posearray_from_list


class PosePublisherNode(Node):
    pose_publisher: Publisher
    prediction_suscriber: Subscription
    dataset: TeleopIcubDataset

    def __init__(self, dataset):
        super().__init__("pose_publisher")
        self.dataset = dataset
        self.idx = 0
        self.frame_counter = 0
        self.pose_publisher = self.create_publisher(
            PoseArray, "/prescyent/position", 10
        )
        self.prediction_suscriber = self.create_subscription(
            PoseArray, "/prescyent/prediction", self.receive_prediction, 10
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.get_logger().info("PosePublisher Node has been started.")

    def receive_prediction(self, prediction: PoseArray):
        self.get_logger().info(f"Received prediction for frame \"{prediction.header.frame_id}\"")

    def publisher_callback(self):
        # as an example we loop over the first test trajectory
        if len(self.dataset.trajectories.test[0]) <= self.idx:
            self.idx = 0
        frame = self.dataset.trajectories.test[0][self.idx]
        self.idx += 1
        # convert Tensor to PoseArray and publish
        pose_array = get_posearray_from_list(frame.tolist())
        pose_array.header.frame_id = str(self.frame_counter)
        pose_array.header.stamp = self.get_clock().now().to_msg()
        self.frame_counter += 1
        self.pose_publisher.publish(pose_array)
        self.get_logger().info(f"Sent position for frame \"{str(self.frame_counter)}\"")


def main():
    # -- Init dataset
    print("Initializing dataset...", end=" ")
    # TODO: use ros params here ?
    subsampling_step: int = 10  # subsampling -> 100 Hz to 10Hz
    history_size = 10  # 1 second
    future_size = 10  # 1 second
    dimensions = None  # None equals ALL dimensions !
    # for TeleopIcub dimension = [0, 1, 2] is waist, right_hand, left_hand
    dataset_config = TeleopIcubDatasetConfig(
        history_size=history_size,
        future_size=future_size,
        dimensions=dimensions,
        subsampling_step=subsampling_step
    )
    dataset = TeleopIcubDataset(dataset_config)
    print("OK")

    # Publish dataset at given rate
    rclpy.init()
    node = PosePublisherNode(dataset)
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
