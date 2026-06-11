FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV GZ_SIM_RESOURCE_PATH=/PX4-Autopilot/Tools/simulation/gz/models:/PX4-Autopilot/Tools/simulation/gz/worlds

RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release wget git build-essential cmake \
    python3-pip python3-dev software-properties-common \
    tmux htop xvfb x11vnc \
    ninja-build exiftool astyle \
    libeigen3-dev libopencv-dev libopencv-contrib-dev \
    libssl-dev libglew-dev libboost-serialization-dev \
    python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-px4-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    kconfiglib jinja2 jsonschema pyros-genmsg \
    packaging toml numpy empy pymavlink

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    --branch v2.4.3 /Micro-XRCE-DDS-Agent && \
    cd /Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DEPROSIMA_BUILD_TESTS=OFF && \
    make -j2 && make install && ldconfig

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
