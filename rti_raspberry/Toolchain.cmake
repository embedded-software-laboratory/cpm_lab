# Define our host system
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

# Define the cross compiler locations
SET(CMAKE_C_COMPILER   /home/janis/toolchains/raspbian-toolchain-gcc-4.7.2-linux64/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /home/janis/toolchains/raspbian-toolchain-gcc-4.7.2-linux64/bin/arm-linux-gnueabihf-g++)

# Define the sysroot path for the RaspberryPi distribution in our tools folder 
SET(CMAKE_FIND_ROOT_PATH /home/janis/toolchains/raspbian-toolchain-gcc-4.7.2-linux64/arm-raspbian-linux-gnueabi/sysroot/)

# Use our definitions for compiler tools
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search for libraries and headers in the target directories only
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
