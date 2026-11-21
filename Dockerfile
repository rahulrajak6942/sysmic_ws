FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV VENV=/sysmic_ws/.venv
ENV PATH="$VENV/bin:$PATH"

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep update

COPY ./fastdds.xml /fastdds.xml

RUN python3 -m venv --system-site-packages $VENV
ENV PATH="/sysmic_ws/.venv/bin:$PATH"

WORKDIR /sysmic_ws

COPY ./requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

COPY ./src ./src
RUN rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/.venv/bin/activate" >> ~/.bashrc

CMD ["bash"]