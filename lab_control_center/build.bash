#!/bin/bash

# Get directory of bash script
BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Get yaml
pushd ../..
if [ ! -d "yaml-cpp" ]; then
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp
    mkdir build
    cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j$(nproc)
fi
popd


mkdir build

cd build
cmake .. -DSIMULATION=$SIMULATION 
make -j$(nproc)
cd ..

# Create launcher link to LCC
escaped_dir=$(printf '%s\n' "${BASH_DIR}" | sed 's:[][\/.^$*]:\\&:g')
sed 's/TEMPLATE_LCC_DIR/'"$escaped_dir"'/g' lab-control-center.desktop > $HOME/.local/share/applications/lab-control-center.desktop