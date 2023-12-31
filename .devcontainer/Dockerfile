##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image -> docker build --target=base
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# automatic buildx arg
ARG TARGETPLATFORM

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=




##############################################
#  Develop image -> docker build --target=dev
##############################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-humble-ament-* \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joy-tester \
  ros-humble-demo-nodes-py \
  ros-humble-demo-nodes-cpp \
  ros-humble-image-transport-plugins \
  ros-humble-rqt-image-view \
  ros-humble-v4l2-camera \
  v4l-utils \
  iputils-ping \
  net-tools \
  nano \
  usbutils \
  jstest-gtk \
  evtest \
  terminator \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install ipython
RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG video $USERNAME 

# Use default bashrc for user
RUN cp /etc/skel/.bashrc /home/$USERNAME

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 

# Give new user rights to its own home dir
RUN chown $USER_UID:$USER_GID /home/$USERNAME


# Switch to cycloneDDS unicast for Docker compatibility
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

# write DDS config based on automatic buildx arg (arm64 -> Jetson, else -> pc)
RUN case "${TARGETPLATFORM}" in \
    "linux/arm64") \
      echo \
      "<?xml version='1.0'?> \n\
      <CycloneDDS> \n\
        <Domain id='any'> \n\
          <General> \n\
            <Interfaces> \n\
              <NetworkInterface name='wlan0'/> \n\
            </Interfaces> \
            <AllowMulticast>false</AllowMulticast> \n\
          </General> \n\
          <Discovery> \n\
            <ParticipantIndex>auto</ParticipantIndex> \n\
            <Peers> \n\
              <Peer Address='localhost'/> \n\
              <Peer Address='192.168.1.110'/> \n\
            </Peers> \n\
          </Discovery> \n\
        </Domain> \n\
      </CycloneDDS>" > /opt/cycloneConfig.xml ;; \  
    *) \
      echo \
      "<?xml version='1.0'?> \n\
      <CycloneDDS> \n\
        <Domain id='any'> \n\
          <General> \n\
            <Interfaces> \n\
              <NetworkInterface name='enp3s0'/> \n\
            </Interfaces> \n\
            <AllowMulticast>false</AllowMulticast> \n\
          </General> \n\
          <Discovery> \n\
            <ParticipantIndex>auto</ParticipantIndex> \n\
            <Peers> \n\
              <Peer Address='localhost'/> \n\
              <Peer Address='192.168.1.111'/> \n\
            </Peers> \n\
          </Discovery> \n\
        </Domain> \n\
      </CycloneDDS>" > /opt/cycloneConfig.xml ;;\
  esac;

RUN echo "export CYCLONEDDS_URI=/opt/cycloneConfig.xml" >> /home/$USERNAME/.bashrc


# fix jstest first time launch error
RUN mkdir -p /home/$USERNAME/.config


ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1


############################################
#  Full image -> docker build --target=full
############################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-desktop \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# Fix setuptools version
RUN pip3 install setuptools==58.2.0

###########################################
#  Full+Gazebo image
###########################################
FROM full AS gazebo

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-gazebo* \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# gazebo no audio card error message fix
RUN touch /etc/asound.conf && echo -e " \
pcm.!default { \n\
  type plug \n\
  slave.pcm 'null' \n\
} \n\
EOF ">> /etc/asound.conf

###########################################
#  Full+Gazebo+Nvidia image
###########################################
FROM gazebo AS gazebo-nvidia

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

