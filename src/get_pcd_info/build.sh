if [ -d "build" ]; then
    rm -r build
fi
if [ -d "out" ]; then
    rm -r out
fi
mkdir build
mkdir out
cd build
cmake ..
make -j4
