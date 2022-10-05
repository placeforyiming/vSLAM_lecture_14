apt-get install qt5-qmake
apt-get install qt5-default
apt-get install libqglviewer-dev-qt5
apt-get install libsuitesparse-dev
apt-get install libcxsparse3
apt-get install libcholmod3

git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20201223_git
mkdir build
cd build
cmake ..
make -j4
make install