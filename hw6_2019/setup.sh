#!/bin/bash

DIR="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
cd "$DIR"

# ---------------------------------------
# Install precompiled 3rd party libraries
# ---------------------------------------

if [[ "$OSTYPE" == "linux-gnu" ]]; then
	sudo apt-get update
	sudo apt-get install curl cmake redis-server unzip build-essential python3-dev python3-tk
	# Install gcc 5 for Ubuntu 14.04:
	# sudo add-apt-repository ppa:ubuntu-toolchain-r/test
	# sudo apt-get update
	# sudo apt-get install gcc-5 g++-5
	# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
elif [[ "$OSTYPE" == "darwin"* ]]; then
	brew install cmake eigen redis
fi

# --------------
# Download Eigen
# --------------

cd lib
if [ ! -f eigen-3.3.4.zip ]; then
	curl -L http://bitbucket.org/eigen/eigen/get/3.3.4.zip -o eigen-3.3.4.zip
fi
unzip -q eigen-3.3.4.zip
rm -rf eigen
mv eigen-eigen-5a0156e40feb eigen
cd ..

# -------------
# Download RBDL
# -------------

cd lib
if [ ! -f rbdl-2.5.0.zip ]; then
	curl -L https://bitbucket.org/rbdl/rbdl/get/v2.5.0.zip -o rbdl-2.5.0.zip
fi
unzip -q rbdl-2.5.0.zip
rm -rf rbdl
mv rbdl-rbdl-379184776089 rbdl
cd rbdl

mkdir -p build
cd build
cmake .. -DCMAKE_MODULE_PATH="$DIR/lib"
make -j4
cd ../../..

# --------------------
# Compile RBDL Wrapper
# --------------------

cd lib
rm -rf build
mkdir -p build
cd build
cmake ..
make -j4
cd ../..
