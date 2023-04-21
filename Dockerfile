FROM ubuntu:jammy AS prereq

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
       curl        \
       gnupg       \
       lsb-release \
       locales     \
       python3-pip \
       libspdlog1 &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN python3 -m pip install -U colcon-common-extensions vcstool

RUN locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8



FROM prereq AS build

RUN apt-get update && apt-get install -y \
      build-essential               \
      cmake                         \
      git                           \
      python3-flake8                \
      python3-flake8-blind-except   \
      python3-flake8-builtins       \
      python3-flake8-class-newline  \
      python3-flake8-comprehensions \
      python3-flake8-deprecated     \
      python3-flake8-docstrings     \
      python3-flake8-import-order   \
      python3-flake8-quotes         \
      python3-pytest                \
      python3-pytest-cov            \
      python3-pytest-repeat         \
      python3-pytest-rerunfailures  \
      python3-rosdep2               \
      python3-setuptools            \
      wget

RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws
RUN wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
RUN vcs import src < ros2.repos
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build

RUN rm /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get remove -y python3-catkin-pkg python3-catkin-pkg-modules && \
    apt-get install -y ros-core-dev

RUN cd src && git clone https://github.com/ros2/ros1_bridge
RUN source install/local_setup.bash && \
    colcon build --packages-select ros1_bridge --cmake-force-configure


FROM prereq AS final

RUN apt-get install -y ros-core-dev
WORKDIR /
RUN mkdir /colcon_ws
COPY --from=build /colcon_ws/install /colcon_ws/install

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
COPY ./dds_profile.xml /dds_profile.xml
