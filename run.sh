docker run --rm --name ros_docker_kinetic_alstar2 \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-env="XAUTHORITY=$XAUTH" \
--volume="$XAUTH:$XAUTH" \
--privileged \
-p $1:5900 -p $2:8888 -e jup_port=$2 -e vnc_port=$1 ros_docker_kinetic
 
 #-v /ros_docker/root/:/root/Desktop