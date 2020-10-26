FROM nvidia/cuda:10.1-base-ubuntu16.04

###########################################
# miniconda
# integrated from https://github.com/ContinuumIO/docker-images/tree/master/miniconda3
###########################################
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH /opt/conda/bin:$PATH

RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates curl git python&& \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean -tipsy && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc

# install jupyterlab
RUN conda install -y jupyterlab
# && conda clean --all

###########################################
# X11 VNC XVFB
# integrated from https://github.com/fcwu/docker-ubuntu-vnc-desktop
###########################################
# taken from https://github.com/fcwu/docker-ubuntu-vnc-desktop
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        software-properties-common \
        curl wget \
        supervisor \
        sudo \
        vim-tiny \
        net-tools \ 
        xz-utils \
        dbus-x11 x11-utils alsa-utils \
        mesa-utils libgl1-mesa-dri \
        lxde x11vnc xvfb \
        gtk2-engines-murrine gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine \
        firefox \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

# tini for subreap                                   
ARG TINI_VERSION=v0.9.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
RUN chmod +x /bin/tini

# set default screen to 1 (this is crucial for gym's rendering)
ENV DISPLAY=:1

RUN apt-get update && apt-get install -y \
        git vim \
        python-numpy python-dev cmake zlib1g-dev libjpeg-dev xvfb ffmpeg xorg-dev python-opengl libboost-all-dev libsdl2-dev swig \
    && rm -rf /var/lib/apt/lists/*

# install gym
RUN cd /opt \
    && git clone https://github.com/openai/gym.git \
    && cd /opt/gym \
    && pip install -e '.[box2d]' \
    && rm -rf ~/.cache/pip 


#---------------------------------------------------------------------
# Install ROS
#---------------------------------------------------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-tf2-sensor-msgs \
    ros-kinetic-geographic-msgs \
    ros-kinetic-move-base-msgs \
    ros-kinetic-ackermann-msgs \
    ros-kinetic-unique-id \
    ros-kinetic-fake-localization \
    ros-kinetic-joy \
    ros-kinetic-imu-tools \
    ros-kinetic-robot-pose-ekf \
    ros-kinetic-grpc \
    ros-kinetic-pcl-ros \
    ros-kinetic-pcl-conversions \
    ros-kinetic-controller-manager \
    ros-kinetic-joint-state-controller \
    ros-kinetic-effort-controllers \
    && apt-get clean

# catkin build tools
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python-pyproj \
    python-catkin-tools \
    && apt-get clean

#Fix locale (UTF8) issue https://askubuntu.com/questions/162391/how-do-i-fix-my-locale-issue
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y locales
RUN locale-gen "en_US.UTF-8"

# Finish
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc


#---------------------------------------------------------------------
# Upgrade Gazebo
#---------------------------------------------------------------------

ARG LIBSDFORMAT_VERSION=5

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-kinetic-gazebo8-plugins \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-gazebo8-ros-control \
    libpcap0.8-dev \
    gazebo8-plugin-base \
    gazebo8-common \
    libsdformat${LIBSDFORMAT_VERSION}-dev \
    libgazebo8-dev \
    gazebo8 \
 && apt-get clean

RUN echo "source /usr/share/gazebo-8/setup.sh" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /home/$username/.bashrc    


RUN apt-get install python-rosdep
RUN rosdep init
RUN sudo rosdep fix-permissions
RUN rosdep update

RUN pip install pyaml rospkg catkin_pkg empy
RUN pip install transformations

WORKDIR /root
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws/src; catkin_init_workspace'
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'
WORKDIR /root/catkin_ws/src

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws/src; git clone https://github.com/ROBOTIS-GIT/turtlebot3.git; \
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git; git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git; \
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git; cd ..; rosdep install --from-paths src -i -y; catkin_make; \
source /root/catkin_ws/devel/setup.bash; rospack profile'

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws/src; git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git; \
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git; git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git; \
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git; git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git; \
git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git; git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git; \
cd ..; catkin_make;'

RUN apt install -y ros-kinetic-moveit
RUN apt-get install ros-kinetic-ar-track-alvar-msgs


# vnc port
EXPOSE 5900
# jupyterlab port
EXPOSE 8888
# startup
COPY image /
ENV HOME /
ENV SHELL /bin/bash

# no password and token for jupyter
ENV JUPYTER_PASSWORD ""
ENV JUPYTER_TOKEN ""

WORKDIR /
# services like lxde, xvfb, x11vnc, jupyterlab will be started

ENTRYPOINT ["/startup.sh"]

#export TURTLEBOT3_MODEL=waffle
#roslaunch turtlebot3_fake turtlebot3_fake.launch
#roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
