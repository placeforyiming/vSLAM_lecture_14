git clone https://github.com/fmtlib/fmt.git
cd fmt
git checkout 8.1.1
mkdir build
cd build
cmake ..
make -j4
make install
cd ..
cd ..

git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build 
cmake ..
make -j4
make install
cd ..
cd ..
