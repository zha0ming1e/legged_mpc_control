# we build this docker based on a docker with ros already installed
FROM ros:noetic-perception-focal

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT 2022-10-03

# fixed a key issue using the method mentioned in https://github.com/PRBonn/bonnetal/commit/0ab883b8356954c3e57a07a73c30bbac8f035a05
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  apt update 


# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=4 ; fi && \
      apt-get update && apt-get install -y \
      vim \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python3-catkin-tools python3-osrf-pycommon \
      python3-matplotlib \
      gfortran \
      autoconf \
      git \
      coinor-libipopt-dev \
      curl \
      libopenmpi-dev \
      apt-utils \
      software-properties-common \
      build-essential \
      libssl-dev

# install python 3.8 
RUN apt-get update && \
    apt-get install -y python3.8
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1  && \
    curl https://bootstrap.pypa.io/pip/get-pip.py -o get-pip.py  && \
    python get-pip.py  && \
    rm get-pip.py  && \
    pip --version
# RUN pip install pybullet
# RUN pip install tensorflow
# RUN pip install numpy
# RUN pip install scipy

# change timezone (this is very important otherwise many ROS topic time will be strange)
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# add OSQP 
# follow https://osqp.org/docs/get_started/sources.html#build-from-sources to install OSQP from sources
WORKDIR /tmp
RUN git clone --recursive https://github.com/oxfordcontrol/osqp
WORKDIR /tmp/osqp
RUN mkdir build 
WORKDIR /tmp/osqp/build
RUN cmake -G "Unix Makefiles" ..  && \
    cmake --build .  && \
    cmake --build . --target install

# add OSQP-python
RUN pip install osqp && \
    apt-get install -y ipython3 wget

# add osqp-eigen
WORKDIR /tmp
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR /tmp/osqp-eigen
RUN mkdir build 
WORKDIR /tmp/osqp-eigen/build
RUN cmake ../  && \
    make  && \
    make install

# lcm dependencies, notice ContactImplicitMPC.jl needs 2.68.3, we try 2.64.6 first
RUN apt-cache madison libglib2.0-dev
RUN apt-get install -y libglib2.0-0=2.64.6-1~ubuntu20.04.4 libglib2.0-dev=2.64.6-1~ubuntu20.04.4
# WORKDIR /tmp
# RUN wget http://archive.ubuntu.com/ubuntu/pool/main/g/glib2.0/libglib2.0-0_2.68.4-1ubuntu1_amd64.deb
# RUN dpkg -i libglib2.0-0_2.68.4-1ubuntu1_amd64.deb
 


# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-ros-control \
      ros-${ROS_DISTRO}-gazebo-ros \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-ros-controllers \
      ros-${ROS_DISTRO}-robot-state-publisher\
      ros-${ROS_DISTRO}-plotjuggler \
      ros-${ROS_DISTRO}-plotjuggler-ros 

# https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
# ssh credentials (test user):
#   user@password 
RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get -y install tzdata

RUN apt-get update \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      make \
      ninja-build \
      cmake \
      autoconf \
      automake \
      locales-all \
      dos2unix \
      rsync \
      tar \
      python3-rospkg \
  && apt-get clean

# To use rosparam load yaml files
RUN pip install pyyaml


# install julia 
ENV JULIA_PATH /usr/local/julia
ENV PATH $JULIA_PATH/bin:$PATH

# https://julialang.org/juliareleases.asc
# Julia (Binary signing key) <buildbot@julialang.org>
ENV JULIA_GPG 3673DF529D9049477F76B37566E3C7DC03D6E495

# https://julialang.org/downloads/
ENV JULIA_VERSION 1.7.3

RUN set -eux; \
	\
	savedAptMark="$(apt-mark showmanual)"; \
	if ! command -v gpg > /dev/null; then \
		apt-get update; \
		apt-get install -y --no-install-recommends \
			gnupg \
			dirmngr \
		; \
		rm -rf /var/lib/apt/lists/*; \
	fi; \
	\
# https://julialang.org/downloads/#julia-command-line-version
# https://julialang-s3.julialang.org/bin/checksums/julia-1.7.3.sha256
	arch="$(dpkg --print-architecture)"; \
	case "$arch" in \
		'amd64') \
			url='https://julialang-s3.julialang.org/bin/linux/x64/1.7/julia-1.7.3-linux-x86_64.tar.gz'; \
			sha256='9b2f4fa12d92b4dcc5d11dc66fb118c47681a76d3df8da064cc97573f2f5c739'; \
			;; \
		'armhf') \
			url='https://julialang-s3.julialang.org/bin/linux/armv7l/1.7/julia-1.7.3-linux-armv7l.tar.gz'; \
			sha256='e9de15c56b9b62727c69d10da4b8e90fa6609d2e94e9cfb9f99128dfb59a8677'; \
			;; \
		'arm64') \
			url='https://julialang-s3.julialang.org/bin/linux/aarch64/1.7/julia-1.7.3-linux-aarch64.tar.gz'; \
			sha256='d9e8b342c80ad1371520ed6d11f55b78aa60746737fbf57ecafd6a23b52dd71d'; \
			;; \
		'i386') \
			url='https://julialang-s3.julialang.org/bin/linux/x86/1.7/julia-1.7.3-linux-i686.tar.gz'; \
			sha256='c1e1a4f9a53affee269c7e740cb8bd46740f9021414459c3ab3bb2c540d9d499'; \
			;; \
		*) \
			echo >&2 "error: current architecture ($arch) does not have a corresponding Julia binary release"; \
			exit 1; \
			;; \
	esac; \
	\
	curl -fL -o julia.tar.gz.asc "$url.asc"; \
	curl -fL -o julia.tar.gz "$url"; \
	\
	echo "$sha256 *julia.tar.gz" | sha256sum --strict --check -; \
	\
	export GNUPGHOME="$(mktemp -d)"; \
	gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$JULIA_GPG"; \
	gpg --batch --verify julia.tar.gz.asc julia.tar.gz; \
	command -v gpgconf > /dev/null && gpgconf --kill all; \
	rm -rf "$GNUPGHOME" julia.tar.gz.asc; \
	\
	mkdir "$JULIA_PATH"; \
	tar -xzf julia.tar.gz -C "$JULIA_PATH" --strip-components 1; \
	rm julia.tar.gz; \
# smoke test
	julia --version

# install cmake 3.20 
WORKDIR /tmp 
RUN apt-get -y install wget libtool
RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
RUN tar -zxvf cmake-3.20.2.tar.gz
RUN ls
WORKDIR /tmp/cmake-3.20.2
RUN ./bootstrap && \
    make -j$(USE_PROC) && \
    make install


# install glib 2.68
RUN apt-get install meson
RUN pip3 install --target=/usr/bin --upgrade meson
WORKDIR /tmp  
RUN wget https://mirror.umd.edu/gnome/sources/glib/2.68/glib-2.68.3.tar.xz
RUN tar xf glib-2.68.3.tar.xz

WORKDIR /tmp/glib-2.68.3
RUN meson --prefix /usr _build  && \
    ninja -C _build  && \
    ninja -C _build install

# Install odrive 
RUN sudo pip3 install --upgrade odrive



# create a non-root user
ARG USERNAME=REXOperator
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# [Optional] Set the default user. Omit if you want to keep the default as root.

USER $USERNAME
CMD /bin/bash

# make unitree workspace
ENV SUPPORT_WS=/home/${USERNAME}/support_files
ENV UNITREE_WS=/home/${USERNAME}/unitree_ws
RUN mkdir -p $SUPPORT_WS 
RUN mkdir -p $UNITREE_WS/src
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init;"
# install unitree sdk dependencies
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd ${SUPPORT_WS}/lcm && \
    git checkout tags/v1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    sudo make install 

# notice we must use v3.8 for go1 
ENV UNITREE_SDK_VERSION=3_8
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd ${SUPPORT_WS}/unitree_legged_sdk && git checkout v3.8.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(USE_PROC) 

WORKDIR $SUPPORT_WS
RUN git clone --depth 1 https://github.com/unitreerobotics/aliengo_sdk.git && \
    cd ${SUPPORT_WS}/aliengo_sdk && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(USE_PROC)     
    
WORKDIR $UNITREE_WS/src
RUN git clone --depth 1 https://github.com/ShuoYangRobotics/unitree_ros.git
WORKDIR $UNITREE_WS
# there are some non ascii code in this file that prevents docker from catkin build the file
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
ENV UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk
ENV ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk
ENV UNITREE_PLATFORM=amd64
# compile just unitree ros unitree_legged_msgs
RUN ls $UNITREE_WS/src/unitree_ros && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build unitree_legged_msgs;"


# casadi 
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/casadi/casadi.git && \
    cd ${SUPPORT_WS}/casadi && git checkout tags/3.5.5 && \
    mkdir ${SUPPORT_WS}/casadi/build && cd ${SUPPORT_WS}/casadi/build && \
    cmake -DWITH_CPLEX=OFF -DWITH_KNITRO=OFF -DWITH_OOQP=OFF -DWITH_SNOPT=OFF ${SUPPORT_WS}/casadi && \
    make -j$(USE_PROC) && \
    sudo make install
# /usr/local/lib/casadi/
# /usr/local/include/casadi

# filter for 
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/ShuoYangRobotics/gram_savitzky_golay.git
WORKDIR ${SUPPORT_WS}/gram_savitzky_golay 
RUN git submodule init && \
    git submodule update && \
    mkdir build 
WORKDIR ${SUPPORT_WS}/gram_savitzky_golay/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ../ && \
    make -j$(USE_PROC) && sudo make install


# config bash 
CMD ["/bin/bash"]
RUN wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -
RUN echo "export ROS_MASTER_URI=http://localhost:11311;export ROS_IP=localhost;export ROS_HOSTNAME=localhost" >> ~/.bashrc && \
    echo "export PATH=/usr/local/julia/bin:${PATH}" >> ~/.bashrc && \
    echo "#unitree config" >> ~/.bashrc && \
    echo "export export UNITREE_SDK_VERSION=${UNITREE_SDK_VERSION}" >> ~/.bashrc && \
    echo "export UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk" >> ~/.bashrc && \
    echo "export ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk" >> ~/.bashrc && \
    echo "export UNITREE_PLATFORM=\"amd64\"" >> ~/.bashrc

# install ocs2 
RUN sudo apt-get update \
    && sudo apt-get install -y libglpk-dev ros-${ROS_DISTRO}-pybind11-catkin \
    liburdfdom-dev liboctomap-dev libassimp-dev doxygen doxygen-latex

ENV OCS2_WS=/home/${USERNAME}/ocs2_ws
RUN mkdir -p $OCS2_WS/src
WORKDIR $OCS2_WS/src
RUN git clone https://github.com/leggedrobotics/ocs2.git && git clone --depth 1 https://github.com/leggedrobotics/pinocchio.git
WORKDIR $OCS2_WS/src/pinocchio
RUN git submodule init && git submodule update --depth 1
WORKDIR $OCS2_WS/src
RUN git clone --depth 1 https://github.com/leggedrobotics/hpp-fcl.git  
WORKDIR $OCS2_WS/src/hpp-fcl
RUN git submodule init && git submodule update --depth 1
WORKDIR $OCS2_WS/src
RUN git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git


RUN sudo apt-get update \
    && sudo apt-get install -y ros-${ROS_DISTRO}-interactive-markers

WORKDIR $OCS2_WS
RUN /bin/bash -c "source ${UNITREE_WS}/devel/setup.bash; catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo; catkin build ocs2_legged_robot_ros"


# final 
RUN /bin/bash -c "source ${OCS2_WS}/devel/setup.bash" && echo "source /home/${USERNAME}/legged_ctrl_ws/devel/setup.bash" >> ~/.bashrc  

# install altro-cpp
# RUN sudo apt-get update \
#     && sudo apt-get install -y libfmt-dev
# WORKDIR $SUPPORT_WS
# RUN git clone https://github.com/bjack205/altro.git && cd altro && \
#    mkdir build && cd build && \
#    cmake -D BUILD_SHARED_LIBS=ON -D CMAKE_INSTALL_PREFIX=/usr/local ../ && cmake --build . -j4 && sudo cmake --install .

ENV SHELL /bin/bash