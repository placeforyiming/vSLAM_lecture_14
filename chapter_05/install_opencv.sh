apt-get update
apt-get install build-essential
apt-get install libgtk2.0-dev
apt-get install pkg-config 
apt-get install libavcodec-dev
apt-get install libavformat-dev 
apt-get install libswscale-dev
apt-get install libvtk7-dev 
apt-get install libjpeg-dev 
apt-get install libtiff-dev 
apt-get install libopenexr-dev 
apt-get install libtbb-dev
apt-get install libcanberra-gtk-module
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4.5
mkdir build
cd build
cmake ..
make -j4
make install
cd ..
cd ..
pkg-config --modversion opencv