FROM ubuntu:xenial

# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# setup keys
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# tempfix for https://github.com/ros-infrastructure/rosdep/pull/454
RUN apt-get update && apt-get install --no-install-recommends -y \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install build/run tools
# installing sudo and not removing apt cache for rosdep install later on
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    byobu \
    openssh-server \
    python-catkin-tools \
    python-wstool \
    wget \
    sudo

# build ros packages
ENV ROS_DISTRO kinetic
ENV WORKSPACE /root/sros
WORKDIR $WORKSPACE
RUN wget https://gist.github.com/codebot/874494b34f5d02af2b24a5a312847b9c/raw -O kinetic-sros.rosinstall \
    && wstool init src kinetic-sros.rosinstall -j8 \
    && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic \
    && catkin config --install \
    && catkin build

RUN ssh-keygen -t rsa -f /root/.ssh/id_rsa -P ""
    # && service ssh start \
    # && ssh-copy-id localhost

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
