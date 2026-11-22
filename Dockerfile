FROM osrf/ros:humble-desktop AS sysmic-dev

COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml
ENV QT_X11_NO_MITSHM=1

COPY ./fastdds.xml /fastdds.xml
RUN uv venv --system-site-packages /sysmic_ws/.venv
ENV PYTHONPATH="/sysmic_ws/.venv/lib/python3.10/site-packages"

WORKDIR /sysmic_ws

COPY ./requirements.txt ./requirements.txt
RUN source /sysmic_ws/.venv/bin/activate && uv pip install --no-cache-dir -r requirements.txt

COPY ./src ./src
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
    && source /sysmic_ws/.venv/bin/activate \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sysmic_ws/.venv/bin/activate" >> ~/.bashrc
RUN echo "source /sysmic_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]