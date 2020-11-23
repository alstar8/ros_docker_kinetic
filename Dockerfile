FROM nvidia/cuda:10.1-base-ubuntu16.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
#ENV PATH /opt/conda/bin:$PATH

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y apt-utils

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

RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates curl git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*    

RUN apt install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa   
RUN apt-get update -y   

RUN apt install -y python3.7 python3.7-dev

RUN apt-get update
COPY requirements/get-pip.py /
RUN python3.7 get-pip.py

RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 10

RUN python3 -V
RUN python -V
RUN pip -V

#RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
#    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
#    rm ~/miniconda.sh && \
#    /opt/conda/bin/conda clean -tipsy && \
#    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
#    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc

# tini for subreap                                   
ARG TINI_VERSION=v0.9.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
RUN chmod +x /bin/tini

# set default screen to 1 (this is crucial for gym's rendering)
ENV DISPLAY=:1

RUN apt-get update && apt-get install -y \
        git vim \
        python3-numpy python3-dev cmake zlib1g-dev libjpeg-dev xvfb ffmpeg xorg-dev python3-opengl libboost-all-dev libsdl2-dev swig \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

# install gym
RUN cd /opt \
    && git clone https://github.com/openai/gym.git \
    && cd /opt/gym \
    && pip install --use-feature=2020-resolver -e '.[box2d]' \
    && rm -rf ~/.cache/pip 


RUN apt-get update
RUN apt-get install -y libprotobuf-dev protobuf-compiler build-essential libssl-dev
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; apt remove --purge cmake; hash -r\\
cd /; wget https://github.com/Kitware/CMake/releases/download/v3.16.5/cmake-3.16.5.tar.gz; tar -zxvf cmake-3.16.5.tar.gz; \
cd cmake-3.16.5; ./bootstrap; make; sudo make install'

RUN apt-get install -y libqt4-dev \
         qt4-dev-tools \ 
         libglew-dev \ 
         glew-utils \ 
         libgstreamer1.0-dev \ 
         libgstreamer-plugins-base1.0-dev \ 
         libglib2.0-dev 

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

#Fix locale (UTF8) issue https://askubuntu.com/questions/162391/how-do-i-fix-my-locale-issue
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y locales
RUN locale-gen "en_US.UTF-8"    

# Finish
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /home/$username/.bashrc

RUN apt-get update

RUN apt-get install -y  python-rosdep python-rospy python-rosgraph python-pyproj \
                        python-catkin-tools python-catkin-pkg python-rospkg \
                        python-rosinstall python-rosinstall-generator

RUN apt-get update
RUN apt-get install ca-certificates

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget -qO - http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update
RUN apt-get install gazebo7 -y

RUN pip install --use-feature=2020-resolver rosdep                        


RUN rosdep init
RUN rosdep update



RUN pip install --use-feature=2020-resolver rospkg catkin_pkg
#RUN apt-get install -y 

WORKDIR /root
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws/src; catkin_init_workspace'
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'
#-- Found PythonInterp: /usr/bin/python2 (found version "2.7.12")
WORKDIR /root/catkin_ws/src

RUN /bin/bash -c "source ~/.bashrc"

#COPY requirements/keyboard /etc/default/keyboard
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y xdotool apt-utils
RUN apt-get install -y  kmod kbd
RUN pip install --use-feature=2020-resolver keyboard

WORKDIR /root

RUN DIR1=$(pwd) && \
    MAINDIR=$(pwd)/3rdparty && \
    mkdir ${MAINDIR} && \
    cd ${MAINDIR} && \
    cd ${MAINDIR} && \
    mkdir eigen3 && \
    cd eigen3 && \
    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz && \
    tar -xzf eigen-3.3.8.tar.gz && \
    cd eigen-3.3.8 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=${MAINDIR}/eigen3_installed/ && \
    make install && \
    cd ${MAINDIR} && \
    wget https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.zip && \
    unzip glew-2.1.0.zip && \
    cd glew-2.1.0/ && \
    cd build && \
    cmake ./cmake  -DCMAKE_INSTALL_PREFIX=${MAINDIR}/glew_installed && \
    make -j4 && \
    make install && \
    cd ${MAINDIR} && \
    #pip install numpy --upgrade
    rm Pangolin -rf && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_PREFIX_PATH=${MAINDIR}/glew_installed/ -DCMAKE_LIBRARY_PATH=${MAINDIR}/glew_installed/lib/ -DCMAKE_INSTALL_PREFIX=${MAINDIR}/pangolin_installed && \
    cmake --build . && \
    cd ${MAINDIR} && \
    rm ORB_SLAM2 -rf && \
    rm ORB_SLAM2-PythonBindings -rf && \
    git clone https://github.com/ducha-aiki/ORB_SLAM2 && \
    git clone https://github.com/ducha-aiki/ORB_SLAM2-PythonBindings && \
    cd ${MAINDIR}/ORB_SLAM2 && \
    sed -i "s,cmake .. -DCMAKE_BUILD_TYPE=Release,cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=${MAINDIR}/eigen3_installed/include/eigen3/ -DCMAKE_INSTALL_PREFIX=${MAINDIR}/ORBSLAM2_installed ,g" build.sh
    #cp ${MAINDIR}/ORB_SLAM2/Vocabulary/ORBvoc.txt ${DIR1}/data/

RUN /bin/bash -c "source ~/.bashrc"
#RUN rm /opt/conda/lib/libz*

#WORKDIR /root
ENV OpenCV_DIR=/opt/ros/kinetic

WORKDIR /  

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws/src; git clone https://github.com/ROBOTIS-GIT/turtlebot3.git; \
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git; git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git; \
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git; git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin.git; \

export GAZEBO_MODEL_PATH=/root/catkin_ws/src/turtlebot3_gazebo_plugin/models; \
export GAZEBO_PLUGIN_PATH=/root/catkin_ws/src/turtlebot3_gazebo_plugin/build; \

git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git; git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git; \
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git; git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git; \
git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git; git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git; \
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git; cd ..; rosdep install --from-paths src -i -y; catkin_make; \
source /root/catkin_ws/devel/setup.bash; rospack profile'

RUN apt-get update
RUN apt install -y ros-kinetic-moveit
RUN apt-get install ros-kinetic-ar-track-alvar-msgs

RUN pip install --use-feature=2020-resolver jupyterlab
RUN pip install --use-feature=2020-resolver opencv-contrib-python transformations

RUN apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
RUN apt-get install -y ros-kinetic-cartographer ros-kinetic-cartographer-ros ros-kinetic-cartographer-ros-msgs ros-kinetic-cartographer-rviz ros-kinetic-hector-mapping ros-kinetic-slam-karto ros-kinetic-frontier-exploration ros-kinetic-navigation-stage




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

#source /opt/ros/kinetic/setup.bash
#source /root/catkin_ws/devel/setup.bash

#export TURTLEBOT3_MODEL=waffle
#roslaunch turtlebot3_fake turtlebot3_fake.launch
#roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
#roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
#rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


#roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
