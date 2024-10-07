# Use the official ROS Noetic image
FROM ros:noetic-ros-base

# Install necessary packages, including Docker CLI
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    ros-noetic-rostest \
    ros-noetic-gazebo-ros-pkgs \
    build-essential \
    git \
    curl \
    wget \
    docker.io \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user and set permissions
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
ENV USER=$USERNAME
ENV HOME=/home/$USERNAME
USER $USERNAME

# Create the catkin workspace
RUN mkdir -p $HOME/catkin_ws

# Expose ROS master port (optional)
EXPOSE 11311

# Set working directory
WORKDIR $HOME

# Entry point
CMD ["bash"]
