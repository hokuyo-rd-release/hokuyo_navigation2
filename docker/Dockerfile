ARG ubuntu_ver=22.04

# --- Stage 1: PCL Builder (Dedicated stage for heavy compilation) ---
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS pcl-builder

# Use ARG to allow limiting parallel build jobs to prevent OOM
ARG MAKEJOBS=4
ENV DEBIAN_FRONTEND=noninteractive
RUN touch /etc/apt/apt.conf.d/99fixbadproxy && \
    echo "Acquire::http::Pipeline-Depth 0; Acquire::http::No-Cache true; Acquire::BrokenProxy true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
    apt-get update && apt-get install -y --no-install-recommends \
    wget git build-essential cmake libeigen3-dev sqlite3 libtiff-dev libcurl4-openssl-dev libboost-all-dev libpng-dev \
    libgl1-mesa-dev libglu1-mesa-dev \
    pkg-config zlib1g-dev libqhull-dev \
    libflann-dev libpcap-dev libusb-1.0-0-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/pcl_build
# PCL 1.14.1 - Optimized build by disabling unnecessary modules
# Added --tries for network stability
RUN wget --tries=10 https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz -O pcl.tar.gz && \
    mkdir pcl && tar -xvf pcl.tar.gz -C pcl --strip-components=1 && \
    cd pcl && \
    cmake -Bbuild \
    -DCMAKE_INSTALL_PREFIX=/opt/pcl \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_visualization=OFF \
    -DBUILD_examples=OFF \
    -DBUILD_apps=OFF \
    -DBUILD_tools=OFF \
    -DBUILD_simulation=OFF \
    -DBUILD_global_tests=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_VTK=ON \
    -DWITH_OPENGL=OFF \
    . && \
    cmake --build build -j${MAKEJOBS} && \
    cmake --install build

# --- Stage 2: Deps Builder (PROJ, yp-spur) ---
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS deps-builder
ARG MAKEJOBS=4

ENV DEBIAN_FRONTEND=noninteractive
RUN touch /etc/apt/apt.conf.d/99fixbadproxy && \
    echo "Acquire::http::Pipeline-Depth 0; Acquire::http::No-Cache true; Acquire::BrokenProxy true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
    apt-get update && apt-get install -y --no-install-recommends \
    wget git build-essential cmake libmodbus-dev libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/deps_build
# yp-spur
RUN git clone https://github.com/hokuyo-rd-release/yp-spur.git && \
    cd yp-spur && mkdir build && cd build && \
    cmake .. && make -j${MAKEJOBS} && make install

# PROJ 9.4.1
RUN wget --tries=10 -q https://download.osgeo.org/proj/proj-9.4.1.tar.gz && \
    tar -zxvf proj-9.4.1.tar.gz && cd proj-9.4.1 && \
    mkdir build && cd build && \
    cmake .. && cmake --build . -j${MAKEJOBS} && cmake --build . --target install

# --- Stage 3: ROS Workspace Builder ---
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS builder

ENV TURTLEBOT3_MODEL=waffle
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# Fix proxy issues and setup locales
RUN touch /etc/apt/apt.conf.d/99fixbadproxy && \
echo "Acquire::http::Pipeline-Depth 0; Acquire::http::No-Cache true; Acquire::BrokenProxy true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
apt-get update && apt-get install -y --no-install-recommends locales && \
locale-gen en_US en_US.UTF-8 && \
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
rm -rf /var/lib/apt/lists/*

# Install basic tools and setup ROS2 repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release tmux vim git wget python3-pip \
    liburdfdom-tools bash-completion software-properties-common \
    qtbase5-dev clang libqt5x11extras5-dev && \
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
rm -rf /var/lib/apt/lists/*

# Copy built libraries from previous stages
COPY --from=pcl-builder /opt/pcl /opt/pcl
COPY --from=deps-builder /usr/local /usr/local

ARG ros2_ver=humble
ARG MAKEJOBS=4
ENV ROS_DISTRO=$ros2_ver

RUN apt-get update && apt-get install -y --no-install-recommends \
ros-$ROS_DISTRO-desktop-full \
ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-sensor-msgs-py \
libvtk9-dev \
python3-rosdep python3-colcon-common-extensions python3-vcstool python3-transforms3d python3-autobahn && \
rosdep init || true && rosdep update && \
rm -rf /var/lib/apt/lists/*

# Setup workspace and build
WORKDIR /home/colcon_ws
RUN mkdir -p src && cd src && \
    git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git && \
    git clone --recursive https://github.com/Hokuyo-aut/hokuyo_navigation2.git && \
    git clone https://github.com/hokuyo-rd-release/icart_mini_driver_ros2.git && \
    cd .. && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    sed -i -E '/(action_msgs|rclpy|ament_.*|rosidl_.*|sensor_msgs_py|autobahn_rce)/d' src/hokuyo_navigation2/requirements.txt && \
    pip3 install --no-cache-dir -r src/hokuyo_navigation2/requirements.txt

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd src/hokuyo_navigation2/hokuyo_slam_ros2 && \
    sed -i 's/add_subdirectory(p2o_viewer)/# add_subdirectory(p2o_viewer)/g' CMakeLists.txt && \
    export CMAKE_PREFIX_PATH=/opt/pcl:$CMAKE_PREFIX_PATH && \
    mkdir build && cmake -Bbuild . -DCMAKE_BUILD_TYPE=Release && cmake --build build -j${MAKEJOBS} && \
    cd /home/colcon_ws && export CMAKE_PREFIX_PATH=/opt/pcl:$CMAKE_PREFIX_PATH && \
    colcon build && \
    colcon build --symlink-install --packages-select icart_mini_driver hokuyo_navigation2 lio_nav2_bringup simple_fastlio_localization


# --- Final Runtime Stage ---
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Fix proxy issues and setup locales (Same as builder stage)
RUN touch /etc/apt/apt.conf.d/99fixbadproxy && \
    echo "Acquire::http::Pipeline-Depth 0; Acquire::http::No-Cache true; Acquire::BrokenProxy true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
    apt-get update && apt-get install -y --no-install-recommends locales curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    locales \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-sensor-msgs-py \
    python3-pip python3-transforms3d python3-autobahn tmux vim git bash-completion xdotool wmctrl bc zenity ufw terminator tree \
    libsqlite3-0 libmodbus5 liburdfdom-tools nautilus && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Copy built artifacts from builder stage
COPY --from=builder /opt/pcl /opt/pcl
COPY --from=builder /usr/local /usr/local
COPY --from=builder /home/colcon_ws/install /home/colcon_ws/install
COPY --from=builder /home/colcon_ws/src /home/colcon_ws/src

ENV LD_LIBRARY_PATH=/opt/pcl/lib:/usr/local/lib:$LD_LIBRARY_PATH

# Create necessary directories
RUN mkdir -p /home/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/{map,rosbag,waypoints}

# Firewall configuration (UFW)
RUN ufw allow 5050/tcp && ufw allow 5000/tcp && ufw allow 5001/tcp && \
    ufw allow 8000/tcp && ufw allow 9000/tcp && ufw allow 9090/tcp && \
    ufw allow 10940/tcp

# Setup environment in .bashrc
RUN echo "source /etc/bash_completion" >> ~/.bashrc && \
echo "export CMAKE_PREFIX_PATH=/opt/pcl:\$CMAKE_PREFIX_PATH" >> ~/.bashrc && \
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
echo "source /home/colcon_ws/install/setup.bash" >> ~/.bashrc && \
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc && \
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

WORKDIR /home/colcon_ws
