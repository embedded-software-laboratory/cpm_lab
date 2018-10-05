export RPI_TOOL_PATH=$(pwd)/cross_compile/tools
cmake . -DCMAKE_TOOLCHAIN_FILE=./cross_compile/Toolchain-rpi.cmake