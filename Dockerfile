
# set ROS distribution
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full

# set environment
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}

ENV EXTRA /opt/extra
RUN mkdir -p ${EXTRA}

# install core packages
RUN apt update && apt-get install -y \
    build-essential \
    software-properties-common \
    cmake \
    curl \
    gazebo \
    git \
    python3-pip \
    lsb-release \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pydantic \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-octomap-mapping \
    wget \
    udev \
 && rm -rf /var/lib/apt/lists/*

# install packages specific to Azure Kinect DK
RUN apt update \
 && apt install -y \
    net-tools \
    openssh-server \
 && apt-get install -y \
    libgtk2.0-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    ffmpeg \
    mlocate \
    locate \
 && rm -rf /var/lib/apt/lists/*

# fix warnings & stuff
RUN apt update \
 && apt install -y \
    nasm \
 && rm -rf /var/lib/apt/lists/*

# install libsound1 (deprecated for 22.04 but necessary for Azure Kinect DK)
RUN curl -sS https://packages.microsoft.com/keys/microsoft.asc | tee /etc/apt/trusted.gpg.d/microsoft.asc \
 && apt-add-repository https://packages.microsoft.com/ubuntu/22.04/prod \
 && apt-get update \
 && wget mirrors.kernel.org/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb \
 && dpkg -i libsoundio1_1.1.0-1_amd64.deb 

# install Azure Kinect SDK via binaries (SDK not available via apt starting from Ubuntu 20.04)
RUN apt update \
 && wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb \
 && yes | dpkg -i libk4a1.4_1.4.1_amd64.deb \
 && wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb \
 && yes | dpkg -i libk4a1.4-dev_1.4.1_amd64.deb \
 && wget  https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb \
 && dpkg -i k4a-tools_1.4.1_amd64.deb \
 && apt update \
 && apt -f install \
 && rm -rf libk4a1.4_1.4.1_amd64.deb libk4a1.4-dev_1.4.1_amd64.deb k4a-tools_1.4.1_amd64.deb libsoundio1_1.1.0-1_amd64.deb

 RUN apt update \
  & cd .. && cd ${EXTRA} \
  && git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git \
  && cp ${EXTRA}/Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/ 

# clone driver for Azure Kinect DK
RUN apt update \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && cd ${ROS2_WS}/src \
 && git clone -b ${ROS_DISTRO} https://github.com/microsoft/Azure_Kinect_ROS_Driver.git \
 && rosdep update \
 && rosdep install --ignore-src --from-paths ${ROS2_WS}/src -r -y \
 && cd ${ROS2_WS} \
 && colcon build \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      #2>/dev/null \
      -DCMAKE_WARN_DEPRECATED=OFF \
      -DCMAKE_POLICY_DEFAULT_CMP0048=NEW \
      -DCMAKE_POLICY_DEFAULT_CMP0064=NEW \
      -DCMAKE_POLICY_DEFAULT_CMP0072=NEW \
#      -Dopengl_gl_preference=GLVND \
      -DCMAKE_C_FLAGS="-Wimplicit-function-declaration" \
      -DCMAKE_CXX_FLAGS="" \
      -DCMAKE_C_FLAGS="-Wno-error" \
      -DCMAKE_CXX_FLAGS="-Wno-error" \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && rm -rf /var/lib/apt/lists/*

# install universal robots binary packages
RUN apt update \
 && apt-get install ros-${ROS_DISTRO}-ur -y \
 && rm -rf /var/lib/apt/lists/*

# add moveit binary
#RUN apt update && apt install ros-humble-moveit -y \
# && apt upgrade -y \
# && . /opt/ros/${ROS_DISTRO}/setup.sh \
# && cd ${ROS2_WS} \
# && colcon build \
#    --packages-select ur_moveit_config \
#    --cmake-args \
#      -DCMAKE_BUILD_TYPE=Release \
# && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
# && rm -rf /var/lib/apt/lists/*

# install moveit 2 binary package
RUN apt update \
 && apt install ros-humble-moveit -y \
 && rm -rf /var/lib/apt/lists/*
    
# add moveit from source
RUN apt update \
 && apt upgrade -y \
 && apt -f install \
 && apt --fix-broken install \
 && rm -rf /var/lib/apt/lists/*
 
# clone octomap rviz2 plugin
RUN apt update \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && cd ${ROS2_WS}/src \
 && git clone -b ros2 https://github.com/OctoMap/octomap_rviz_plugins.git \
 && rosdep update \
 && rosdep install --ignore-src --from-paths ${ROS2_WS}/src -r -y \
 && cd ${ROS2_WS} \
 && colcon build \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && rm -rf /var/lib/apt/lists/*

# build workspace
RUN cd ${ROS2_WS} \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && rosdep update \
 && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
 && colcon build \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY /entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
