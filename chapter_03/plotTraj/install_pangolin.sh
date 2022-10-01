# Get Pangolin
cd plotTraj
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
apt-get install libglew-dev
git checkout v0.6
mkdir build
cd build
cmake ..
make -j4
make install