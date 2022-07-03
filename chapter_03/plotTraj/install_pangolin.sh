# Get Pangolin
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
# Install dependencies (as described above, or your preferred method)
#./scripts/install_prerequisites.sh recommended
apt-get install libglew-dev
git checkout v0.6
#./scripts/install_prerequisites.sh recommended
mkdir build
cd build
cmake ..
make -j4
make install