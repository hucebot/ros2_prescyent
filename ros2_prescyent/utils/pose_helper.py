"""functions to manipulate Pose objects"""
from typing import List

from geometry_msgs.msg import PoseArray, Pose


DEFAULT_FLOAT = -1.0


def get_pose_from_list(pose_list: List[float]) -> Pose:
    """Creates a Pose object from a list of pose values"""
    pose = Pose()
    # 7 elements in Pose: Point x, y, z + Quaternion x, y, z, w
    pose.position.x = pose_list[0] if len(pose_list) > 0 else DEFAULT_FLOAT
    pose.position.y = pose_list[1] if len(pose_list) > 1 else DEFAULT_FLOAT
    pose.position.z = pose_list[2] if len(pose_list) > 2 else DEFAULT_FLOAT
    pose.orientation.x = pose_list[3] if len(pose_list) > 3 else DEFAULT_FLOAT
    pose.orientation.y = pose_list[4] if len(pose_list) > 4 else DEFAULT_FLOAT
    pose.orientation.z = pose_list[5] if len(pose_list) > 5 else DEFAULT_FLOAT
    pose.orientation.w = pose_list[6] if len(pose_list) > 6 else DEFAULT_FLOAT
    return pose


def get_list_from_pose(pose: Pose) -> List[float]:
    pose_list = []
    if pose.position.x != DEFAULT_FLOAT:
        pose_list.append(pose.position.x)
    if pose.position.y != DEFAULT_FLOAT:
        pose_list.append(pose.position.y)
    if pose.position.z != DEFAULT_FLOAT:
        pose_list.append(pose.position.z)
    if pose.orientation.x != DEFAULT_FLOAT:
        pose_list.append(pose.orientation.x)
    if pose.orientation.y != DEFAULT_FLOAT:
        pose_list.append(pose.orientation.y)
    if pose.orientation.z != DEFAULT_FLOAT:
        pose_list.append(pose.orientation.z)
    if pose.orientation.w != DEFAULT_FLOAT:
        pose_list.append(pose.orientation.w)
    return pose_list


def get_posearray_from_list(frame: List[List[float]]) -> PoseArray:
    pose_array = PoseArray()
    for pose_list in frame:
        pose = get_pose_from_list(pose_list)
        pose_array.poses.append(pose)
    return pose_array


def get_list_from_posearray(pose_array: PoseArray) -> List[List[float]]:
    list_pose_array = []
    for pose in pose_array.poses:
        list_pose_array.append(get_list_from_pose(pose))
    return list_pose_array
