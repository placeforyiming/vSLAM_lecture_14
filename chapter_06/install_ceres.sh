apt-get install libgoogle-glog-dev
apt-get install libgflags-dev
apt-get install libatlas-base-dev
apt-get install libeigen3-dev
apt-get install libsuitesparse-dev

git clone https://github.com/ceres-solver/ceres-solver
cd ceres-solver 
git checkout 1.14.0
sed -i '58,59 s/^/#/' CMakeLists.txt
mkdir build
cd build 
cmake ..
make -j4
make test
make install