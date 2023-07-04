"""Ros2 topic suscriber to current positions
that predicts and publishes the N next ones"""
import copy
from typing import List

import torch
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node, ParameterDescriptor, ParameterNotDeclaredException
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from prescyent.auto_predictor import get_predictor_from_path
from prescyent.predictor.base_predictor import BasePredictor
from prescyent.utils.errors import PredictorNotFound
from ros2_prescyent.utils.pose_helper import (get_list_from_pose, get_list_from_posearray,
                                              get_posearray_from_list)
from ros2_prescyent.utils.time_helper import get_duration_from_frequency, update_time


class PredictorNode(Node):
    predictor: BasePredictor
    _pose_buffer: List[Pose]
    prediction_publisher: Publisher
    pose_suscriber: Subscription

    def __init__(
        self,
        predictor_path: str = None,
        future_size: int = None,
        history_size: int = None,
        predictor_frequency: int = None
    ):
        super().__init__("prescyent_predictor")
        self._declare_parameters(predictor_path, future_size, history_size, predictor_frequency)
        self.add_on_set_parameters_callback(self.ros2param_update_callback)
        try:
            self.predictor = get_predictor_from_path(self.predictor_path)
        except PredictorNotFound:
            self.get_logger().error(
                f"Could not find a predictor at {self.predictor_path}"
            )
            exit(1)
        self._pose_buffer = list()
        self.prediction_publisher = self.create_publisher(
            PoseArray, "/prescyent/prediction", 10
        )
        self.pose_suscriber = self.create_subscription(
            PoseArray, "/prescyent/position", self.receive_pose, 10
        )
        self.get_logger().info("Predictor Node has been started.")

    @property
    def history_size(self) -> int:
        return self._get_ros2_param("history_size")

    @property
    def future_size(self) -> int:
        return self._get_ros2_param("future_size")

    @property
    def predictor_path(self) -> str:
        return self._get_ros2_param("predictor_path")

    @property
    def predictor_frequency(self) -> int:
        return self._get_ros2_param("predictor_frequency")

    def _get_ros2_param(self, param_name):
        try:
            return self.get_parameter(param_name).value
        except ParameterNotDeclaredException:
            return None

    def ros2param_update_callback(self, parameters: List[Parameter]):
        for p in parameters:
            self.get_logger().info(f"Received param {p.name} = {p.value}")
            if p.name == "predictor_path":
                try:
                    self.predictor = get_predictor_from_path(p.value)
                except PredictorNotFound:
                    self.get_logger().error(f"Could not find a predictor at {p.value}")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def _declare_parameters(self, predictor_path, future_size, history_size, predictor_frequency):
        default_predictor_path = predictor_path if predictor_path else ""
        default_future_size = future_size if future_size else 10
        default_history_size = history_size if history_size else 10
        default_predictor_frequency = predictor_frequency if predictor_frequency else 10
        self.get_logger().debug("Declaring ros params")
        self.declare_parameter(
            "predictor_path",
            default_predictor_path,
            ParameterDescriptor(description="Path to the loaded predictor"),
        )
        self.declare_parameter(
            "future_size",
            default_future_size,
            ParameterDescriptor(description="Size of the input sequence"),
        )
        self.declare_parameter(
            "history_size",
            default_history_size,
            ParameterDescriptor(description="Size of the output sequence"),
        )
        self.declare_parameter(
            "predictor_frequency",
            default_predictor_frequency,
            ParameterDescriptor(
                description="The amount of Pose to skip while building the input sequence"
            ),
        )
        self.get_logger().info(
            f"Using ros2 param: predictor_path = {self.predictor_path}"
        )
        self.get_logger().info(f"Using ros2 param: history_size = {self.history_size}")
        self.get_logger().info(f"Using ros2 param: future_size = {self.future_size}")
        self.get_logger().info(f"Using ros2 param: predictor_frequency = {self.predictor_frequency}")

    def get_tensor_from_trajectory(self, pose_array_list: List[PoseArray]) -> torch.Tensor:
        pose_tensor = []
        for pose_array in pose_array_list:
            for pose in pose_array.poses:
                pose_tensor.append(get_list_from_pose(pose))
        return torch.Tensor(pose_tensor)

    def get_trajectory_from_tensor(
        self, tensor: torch.Tensor, history: List[PoseArray]
    ) -> PoseArray:
        """Tensors are of size (seq_size, num_points, num_dims)
        We return a Trajectory of format [PoseArray[Pose]]"""
        sequence = tensor.tolist()
        seq_len = len(sequence)
        prediction = sequence[-1]
        last_stamp = history[-1].header.stamp
        last_frame_id = int(history[-1].header.frame_id)
        pose_array = get_posearray_from_list(prediction)
        secs, nanosecs = get_duration_from_frequency(self.predictor_frequency)
        secs, nanosecs = secs * seq_len, nanosecs * seq_len
        pose_array.header.stamp = update_time(time=last_stamp,
                                                seconds=secs,
                                                nanoseconds=nanosecs)
        pose_array.header.frame_id = str(last_frame_id + seq_len)
        return pose_array

    def receive_pose(self, pose: Pose):
        self._pose_buffer.append(pose)
        if len(self._pose_buffer) > self.history_size:
            self._pose_buffer.pop(0)
        if len(self._pose_buffer) == self.history_size:
            pose_buffer = copy.deepcopy(self._pose_buffer)
            history = torch.Tensor([get_list_from_posearray(pose) for pose in pose_buffer])
            prediction = self.predictor.predict(history, future_size=self.future_size)
            self.prediction_publisher.publish(
                self.get_trajectory_from_tensor(prediction, pose_buffer)
            )
