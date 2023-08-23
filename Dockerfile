FROM ubuntu:jammy as prereq

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y &&\
    apt-get install -y \
        curl \
        gnupg \
        lsb-release \
        locales \
        python3-pip &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN python3 -m pip install -U colcon-common-extensions vcstool

RUN locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

FROM prereq as build

RUN apt-get update -y &&\
    apt-get install -y \
      build-essential \
      cmake \
      git \
      python3-flake8 \
      python3-flake8-blind-except \
      python3-flake8-builtins \
      python3-flake8-class-newline \
      python3-flake8-comprehensions \
      python3-flake8-deprecated \
      python3-flake8-docstrings \
      python3-flake8-import-order \
      python3-flake8-quotes \
      python3-pytest \
      python3-pytest-cov \
      python3-pytest-repeat \
      python3-pytest-rerunfailures \
      python3-rosdep2 \
      python3-setuptools \
      wget

RUN mkdir -p /ros2_iron/src
WORKDIR /ros2_iron
RUN wget https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos
RUN vcs import src < ros2.repos
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --rosdistro iron \
      --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build

RUN rm /etc/apt/sources.list.d/ros2.list &&\
    apt-get update -y &&\
    apt remove -y \
        python3-catkin-pkg python3-catkin-pkg-modules &&\
    apt-get install -y \
        ros-core-dev \
        libmap-msgs-dev \
        ros-map-msgs \
        libmove-base-msgs-dev \
        ros-move-base-msgs \
        libpcl-msgs-dev \
        ros-pcl-msgs \
        libtf2-msgs-dev \
        ros-tf2-msgs

RUN cd src && git clone https://github.com/ros2/ros1_bridge
RUN /bin/bash -c ". /ros2_iron/install/local_setup.bash &&\
                  colcon build"

FROM prereq as final

RUN apt-get install -y ros-core-dev liblttng-ust1 libspdlog1
WORKDIR /
RUN mkdir /ros2_iron
COPY --from=build /ros2_iron/install /ros2_iron/install

COPY ./dds_profile.xml /dds_profile.xml
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=iron
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
