# vSLAM_lecture_14

Code implementation of the book  https://github.com/gaoxiang12/slambook2 . I skip some chapters that not interested.

Prepare the docker environment:
'
docker pull ros:noetic

sudo xhost +local:root

docker run -it  --name vslam_14  --mount type=bind,source=/home/yiming/Projects,target=/workspace --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros:melodic bash

apt-get update

apt-get install ros-noetic-rviz
'

Test the ros docker env:

source /opt/ros/noetic/setup.bash

rosrun rviz rviz

roscore &
