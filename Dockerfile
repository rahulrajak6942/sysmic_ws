FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rosdep update

COPY ./fastdds.xml /fastdds.xml

WORKDIR /sysmic_ws
COPY ./src ./src
COPY ./requirements.txt ./requirements.txt

RUN python3 -m venv .venv && source .venv/bin/activate
RUN pip install --no-cache-dir -r requirements.txt

RUN rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/.venv/bin/activate" >> ~/.bashrc

CMD ["bash"]