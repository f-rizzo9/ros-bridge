ARG CARLA_VERSION=0.9.13
ARG ROS_DISTRO=noetic

FROM carlasim/carla:$CARLA_VERSION as carla

FROM ros:$ROS_DISTRO-ros-base

ARG CARLA_VERSION=0.9.13
ARG ROS_DISTRO=noetic

ENV CARLA_VERSION=$CARLA_VERSION
ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /opt/carla-ros-bridge/src
WORKDIR /opt/carla-ros-bridge

COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

COPY requirements.txt /opt/carla-ros-bridge
COPY install_dependencies.sh /opt/carla-ros-bridge
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    bash /opt/carla-ros-bridge/install_dependencies.sh; \
    if [ "$CARLA_VERSION" = "0.9.10" ] || [ "$CARLA_VERSION" = "0.9.10.1" ]; then wget https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg -P /opt/carla/PythonAPI/carla/dist; fi; \
    echo "export PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3/dist-packages:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$ROS_PYTHON_VERSION.)" >> /opt/carla/setup.bash; \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash'

COPY . /opt/carla-ros-bridge/src/
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    if [ "$ROS_VERSION" == "2" ]; then colcon build; else catkin_make install; fi'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

# Install additional packages
RUN apt-get update && \
    apt-get install -y nano ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-common-plugins && \
    rm -rf /var/lib/apt/lists/*

# Give execution permissions to the setup.bash script
RUN chmod +x /opt/carla/setup.bash

# Source the ROS setup and Carla setup scripts in the bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source /opt/carla/setup.bash' >> ~/.bashrc
RUN echo 'source /opt/carla-ros-bridge/devel/setup.bash' >> ~/.bashrc