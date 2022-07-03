# vSLAM_lecture_14

1. pull the ros docker image with :

docker pull ros:noetic

2. start and get into the docker container :

sudo xhost +local:root

docker run -it  --name vslam_14  --mount type=bind,source=/home/yiming/Projects,target=/workspace --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros:melodic bash

3. install rviz

apt-get update

apt-get install ros-noetic-rviz

4. test

source /opt/ros/noetic/setup.bash

rosrun rviz rviz

roscore &
