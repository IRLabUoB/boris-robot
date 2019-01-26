FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# Requirement:
# install nvidia-docker,
# container must be started with nvidia-docker

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# build setups follows indigo-ros-core -> indigo-ros-base -> indigo-desktop-full
# only difference is not based on ubuntu:trusty

ENV DEBIAN_FRONTEND noninteractive

#ARG http_proxy
#ARG https_proxy

#ENV http_proxy=$http_proxy
#ENV https_proxy=$https_proxy


# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
&& rm -rf /var/lib/apt/lists/*


# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.2-0* \
&& rm -rf /var/lib/apt/lists/*

# install ros desktop full packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*



# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /


# installs avahi for local network discovery (required for using the real robot)
RUN apt-get update && apt-get install -y \
    avahi-daemon avahi-utils


# Dependencies for baxter and sawyer simulator
RUN apt-get update && \ 
    apt-get -y install sudo apt-utils python-pip python-scipy libprotobuf-dev protobuf-compiler \
                       python-rosinstall python-rosinstall-generator python-wstool build-essential \
                       ros-kinetic-ros-controllers ros-kinetic-ros-control ros-kinetic-moveit git-core \
                       ros-kinetic-gazebo-ros-control && \
    rm -rf /var/lib/apt/lists/*
 

# RUN pip install --upgrade pip && \
#     pip install protobuf && \
#     pip install pandas

#six PySide
# RUN pip install --upgrade pip==9.0.3 && \
#     pip install protobuf numpy decorator ipython jupyter matplotlib Pillow scipy pandas


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]