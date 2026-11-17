FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && rosdep update

COPY ./fastdds.xml /fastdds.xml

WORKDIR /sysmic_ws
COPY ./src ./src

RUN rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]