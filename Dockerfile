# Base: Ubuntu 20.04 focal + ros-foxy-desktop
# built from github.com/osrf/docker_images/tree/master/ros/foxy/ubuntu/focal/desktop
FROM osrf/ros:foxy-desktop

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# default user: should be overridden on command line of 'docker run'
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install nvidia driver
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    nvidia-driver-460 \
    && rm -rf /var/lib/apt/lists/*

# install the tango icon theme and use in place of hicolor
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    tango-icon-theme \
    && rm -rf /var/lib/apt/lists/* \
    && rm -r /usr/share/icons/hicolor/ \
    && mv /usr/share/icons/Tango/ /usr/share/icons/hicolor

# install ros development tools
RUN apt update && apt install -q -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-tk \
    python3-vcstool \
    wget \
    && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

# install secret gazebo dependency for tutorial
RUN apt update && apt install -q -y --no-install-recommends \
    ros-foxy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# setup working directory
WORKDIR /home/${USERNAME}/ws/src

# add user to ssh group for joystick
RUN usermod -aG ssh $USERNAME

# setup default user, when entering docker container
RUN chown -R $USERNAME:$USERNAME /home/${USERNAME}
USER $USERNAME
RUN sudo usermod -aG sudo $USERNAME
