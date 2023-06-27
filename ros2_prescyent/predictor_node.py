"""Ros2 topic suscriber to current positions
that predicts and publishes the N next ones"""
import copy
from typing import List

import torch
from geometry_msgs.msg import PoseArray, Pose
from prescyent_msgs.msg import Trajectory
from prescyent.predictor.base_predictor import BasePredictor
from prescyent.auto_predictor import get_predictor_from_path
from prescyent.utils.errors import PredictorNotFound
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node, ParameterDescriptor, ParameterNotDeclaredException
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription


def get_pose_from_list(pose_list: List[float]) -> Pose:
    """Creates a Pose object from a list of pose values"""
    pose = Pose()
    # 7 elements in Pose: Point x, y, z + Quaternion x, y, z, w
    pose.position.x = pose_list[0] if len(pose_list) >= 0 else None
    pose.position.y = pose_list[1] if len(pose_list) >= 1 else None
    pose.position.z = pose_list[2] if len(pose_list) >= 2 else None
    pose.orientation.x = pose_list[3] if len(pose_list) >= 3 else None
    pose.orientation.y = pose_list[4] if len(pose_list) >= 4 else None
    pose.orientation.z = pose_list[5] if len(pose_list) >= 5 else None
    return pose


def get_list_from_pose(pose: Pose) -> List[float]:
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


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
        time_step: int = None,
    ):
        super().__init__("prescyent_predictor")
        self._declare_parameters(predictor_path, future_size, history_size, time_step)
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
            Pose, "/prescyent/positions", self.receive_pose, 10
        )
        self.get_logger().info("Predictor Node has been started.")

    @property
    def history_size(self):
        return self._get_ros2_param("history_size")

    @property
    def time_step(self):
        return self._get_ros2_param("time_step")

    @property
    def future_size(self):
        return self._get_ros2_param("future_size")

    @property
    def predictor_path(self):
        return self._get_ros2_param("predictor_path")

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

    def _declare_parameters(self, predictor_path, future_size, history_size, time_step):
        default_predictor_path = predictor_path if predictor_path else ""
        default_future_size = future_size if future_size else 10
        default_history_size = history_size if history_size else 10
        default_time_step = time_step if time_step else 10
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
            "time_step",
            default_time_step,
            ParameterDescriptor(
                description="The amount of Pose to skip while building the input sequence"
            ),
        )
        self.get_logger().info(
            f"Using ros2 param: predictor_path = {self.predictor_path}"
        )
        self.get_logger().info(f"Using ros2 param: history_size = {self.history_size}")
        self.get_logger().info(f"Using ros2 param: future_size = {self.future_size}")
        self.get_logger().info(f"Using ros2 param: time_step = {self.time_step}")

    def get_tensor_from_trajectory(self, poses: List[PoseArray]):
        pose_tensor = []
        return pose_tensor

    def get_trajectory_from_tensor(
        self, tensor: torch.Tensor, history: List[PoseArray]
    ) -> List[PoseArray]:
        """Tensors are of size (seq_size, num_points, num_dims)
        We return a Trajectory of format [PoseArray[Pose]]"""
        time_step = self.get_parameter("time_step").get_parameter_value().integer_value
        sequence = tensor.to_list()
        trajectory = Trajectory()
        last_stamp = history[-1].header.stamp
        last_frame_id = history[-1].header.frame_id
        for pose_array_list in sequence:
            pose_array = PoseArray()
            for pose_list in pose_array_list:
                pose = get_pose_from_list(pose_list)
                pose_array.poses.append(pose)
            last_stamp = last_stamp + time_step
            last_frame_id += 1
            pose_array.header.stamp = last_stamp
            pose_array.header.frame_id = last_frame_id
            trajectory.pose_array_sequence.append(pose_array)
        return trajectory

    def receive_pose(self, pose: Pose):
        self._pose_buffer.append(pose)
        history_size = (
            self.get_parameter("history_size").get_parameter_value().integer_value
        )
        future_size = (
            self.get_parameter("future_size").get_parameter_value().integer_value
        )
        if len(self._pose_buffer) == history_size:
            pose_buffer = copy.deepcopy(self._pose_buffer)
            history = self.get_tensor_from_trajectory(pose_buffer)
            self._pose_buffer = list()
            prediction = self.predictor(history, future_size=future_size)
            self.prediction_publisher.publish(
                self.get_trajectory_from_tensor(prediction, pose_buffer)
            )
