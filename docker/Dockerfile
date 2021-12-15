FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
LABEL maintainer="YinWu<yinwu.life@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/sh", "-c"]

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
    # dependencies
    apt-utils sudo lsb-release locales bash-completion tzdata gosu \
    # network
    wget curl net-tools \
    # development
    git build-essential vim  pkg-config \
    # c++
    gcc g++ cmake && \
    rm -rf /var/lib/apt/lists/*\
    # for slam development
    libceres-dev \
    # for VIO lesson
    libdw-dev

RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV USER ubuntu
# ------ set up ROS ------
COPY packages/ros_install.sh /home/ubuntu/init/
RUN chmod +x /home/ubuntu/init/ros_install.sh && /home/ubuntu/init/ros_install.sh
