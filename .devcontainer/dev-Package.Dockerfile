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
