FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    git \
    wget \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install ROS tools
RUN pip3 install -U \
    argcomplete \
    vcstool \
    rosdep

# Only update rosdep (init already done in base image)
RUN rosdep update

# Source setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["bash"]

