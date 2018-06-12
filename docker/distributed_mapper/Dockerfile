# docker build -t awilby/distributed_mapper .
# docker export distributed_mapper| gzip -c > distributed_mapper.tgz
# docker import distributed_mapper < distributed_mapper.tgz

############################################################
# Dockerfile to build DistributedMapper images
# Based on Ubuntu
############################################################

FROM ubuntu:16.04
MAINTAINER Cognitive Robotics "http://cogrob.org/"

RUN useradd -ms /bin/bash distmapper -G sudo

USER root

RUN apt-get update

# Install Prerequisites
RUN apt-get install -y \
    build-essential \
    g++ \
    cmake \
    libboost-all-dev

# Install CLI Tools
RUN apt-get install -y \
    wget \
    curl \
	screen \
	byobu \
	fish \
	git \
	nano \
	glances


CMD ["bash"]

# Build & Install GTSAM
############################################################

WORKDIR /home/distmapper

RUN git clone https://bitbucket.org/gtborg/gtsam  \
    && cd gtsam \
    && git checkout b7c695fa71efd43b40972eec154df265617fc07d -b dist-mapper

RUN cd /home/distmapper/gtsam/ \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8 \
    && make install


# Build Distributed Mapper
############################################################

WORKDIR /home/distmapper

RUN git clone https://github.com/CogRob/distributed-mapper.git

RUN cd /home/distmapper/distributed-mapper/cpp/ \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8


# Run Distributed Mapper
############################################################

WORKDIR /home/distmapper/distributed-mapper/cpp/build

ENTRYPOINT ["./runDistributedMapper"]
