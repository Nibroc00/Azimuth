# syntax=docker/dockerfile:1
FROM arm64v8/ros:noetic-robot

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src \
    && mkdir /home/rosbags

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        htop \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
        geographiclib-tools \
        libgeographic-dev \
	&& geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink \
    pynmeagps \
    numpy \
    numpy-quaternion \
    geographiclib

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-control-toolbox

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && rosdep update \
    && catkin init \
    && wstool init src

RUN cd /home/catkin_ws/src \
    && git clone https://github.com/Nibroc00/Azimuth.git \
    && cd ..


# Build the catkin workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin build -j1

# Overlay the ROS environment on top of the existing environment
COPY ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
