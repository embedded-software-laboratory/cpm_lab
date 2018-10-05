export RPI_TOOL_PATH=$(pwd)/cross_compile/tools
export MYPWD=$(pwd)
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$MYPWD/cross_compile/Toolchain-rpi.cmake
make
cd ..