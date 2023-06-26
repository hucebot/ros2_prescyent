#!/usr/bin/env python3
from argparse import ArgumentParser

import rclpy

from prescyent.auto_predictor import get_predictor_from_path
from ros2_prescyent.predictor_node import PredictorNode


def main(predictor=None, history_size=10, future_size=10, time_step=10):
    rclpy.init()
    predictor = get_predictor_from_path(predictor)
    node = PredictorNode(predictor=predictor,
                         history_size=history_size,
                         future_size=future_size,
                         time_step=time_step)
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--predictor", default=0)
    parser.add_argument("--history_size", default=10)
    parser.add_argument("--future_size", default=10)
    parser.add_argument("--time_step", default=10)
    args = parser.parse_args()
    main(**vars(args))
