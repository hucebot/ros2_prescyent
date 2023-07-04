#!/usr/bin/env python3
from argparse import ArgumentParser

import rclpy

from ros2_prescyent.nodes.predictor_node import PredictorNode


def main(predictor_path=None, history_size=10, future_size=10, predictor_frequency=10):
    rclpy.init()
    node = PredictorNode(
        predictor_path=predictor_path,
        future_size=future_size,
        history_size=history_size,
        predictor_frequency=predictor_frequency,
    )
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--predictor_path", default='')
    parser.add_argument("--history_size", default=10)
    parser.add_argument("--future_size", default=10)
    parser.add_argument("--predictor_frequency", default=10)
    args = parser.parse_args()
    main(**vars(args))
