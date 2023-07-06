# syntax=docker/dockerfile:1

FROM ros:humble
ENV ROS_DISTRO humble
ENV ROS_PYTHON_VERSION 3
ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"


RUN apt -q -qq update && apt install -y --allow-unauthenticated  python3-pip
RUN apt install python-is-python3
RUN python -m pip install --upgrade pip

WORKDIR /root/
RUN mkdir -p ros2_ws/src/ros2_prescyent
WORKDIR /root/ros2_ws/src/ros2_prescyent
COPY prescyent prescyent
WORKDIR /root/ros2_ws/src/ros2_prescyent/prescyent
RUN python -m pip install -e .

WORKDIR /root/ros2_ws/src/ros2_prescyent
COPY launch launch
COPY resource resource
COPY ros2_prescyent ros2_prescyent
COPY test test
COPY package.xml package.xml
COPY setup.py setup.py
COPY setup.cfg setup.cfg

WORKDIR /root/ros2_ws
RUN rosdep install -i --from-path src --rosdistro humble -yr
RUN colcon build

RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
RUN colcon build

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
# CMD ["ros2 launch ros2_prescyent ros2_predict.launch.py"]
