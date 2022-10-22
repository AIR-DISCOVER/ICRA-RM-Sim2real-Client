##########################################################################
FROM ros:noetic-ros-core AS ros-base

##########################################################################
FROM tb5zhh/ubuntu-ca-certificates:focal AS base
SHELL ["/bin/bash", "-c"]
RUN sed -i "s@http://.*archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list
RUN sed -i "s@http://.*security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list

RUN echo 'Etc/UTC' > /etc/timezone && \
     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
     apt-get update && \
     apt-get install -y --no-install-recommends \
     tzdata dirmngr wget build-essential libc-dev cmake python3-pip \
     sudo stow ninja-build git software-properties-common g++ gnupg2 && \
     rm -rf /var/lib/apt/lists/* && apt-get clean

COPY --from=ros-base /opt/ros /opt/ros

### Required packages
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros1-latest.list 


ADD cartographer /opt/workspace/src/cartographer
ADD cartographer_ros /opt/workspace/src/cartographer_ros
ADD abseil-cpp /opt/workspace/abseil-cpp
WORKDIR /opt/workspace
RUN src/cartographer/scripts/install_abseil.sh
ADD proxy.sh /opt/proxy.sh
ADD direct.sh /opt/direct.sh

RUN pip3 install rosdep rosinstall -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN source /opt/proxy.sh && rosdep init && DEBIAN_FRONTEND=noninteractive \
     rosdep update -q --rosdistro noetic && apt-get update -q && \
     ROS_OS_OVERRIDE=ubuntu:20.04:focal \
     DEBIAN_FRONTEND=noninteractive \
     rosdep install -q --from-paths src --ignore-src --rosdistro noetic -y && \
     rm -rf /var/lib/apt/lists/* && \
     apt-get clean && source /opt/direct.sh

RUN pip3 install rosdep rosinstall empy defusedxml netifaces pandas seaborn \
     -i https://pypi.tuna.tsinghua.edu.cn/simple

RUN source /opt/ros/noetic/setup.bash && catkin_make_isolated --install --use-ninja

ADD sim2real_ep /opt/ep_ws/src
WORKDIR /opt/ep_ws
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
RUN apt-get update -q && \
     source /opt/workspace/devel_isolated/setup.bash && \
     ROS_OS_OVERRIDE=ubuntu:20.04:focal \
     DEBIAN_FRONTEND=noninteractive \        
     rosdep install -q --from-paths src --ignore-src --rosdistro noetic -y && \
     rm -rf /var/lib/apt/lists/* && \
     apt-get clean

RUN source /opt/workspace/devel_isolated/setup.bash && catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF
ENV ENV_ROBOT_MODE=sim