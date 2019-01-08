#!/bin/sh
# Script for installing Caffe with cuDNN support on Jetson TX2 Development Kits
# Requirements:
# Jetson TX2 flashed with JetPack 3.3.
sudo add-apt-repository universe
sudo apt-get update -y
/bin/echo -e "\e[1;32mLoading Caffe Dependencies .........\e[0m"
sudo apt-get install cmake -y
#
#
sudo apt-get install libboost-system-dev libboost-thread-dev libgflags-dev libgoogle-glog-dev libhdf5-serial-dev libleveldb-dev liblmdb-dev libopencv-dev libsnappy-dev python-all-dev python-dev python-h5py python-matplotlib python-numpy python-opencv python-pil python-pip python-pydot python-scipy python-skimage python-sklearn    
#
#
sudo apt-get install python-setuptools
sudo apt-get install autoconf automake libtool curl make g++ unzip
sudo apt-get install protobuf-compiler      
#
#
/bin/echo -e "\e[1;32mLoading Protobuf .........\e[0m"
export PROTOBUF_ROOT=~/protobuf
git clone https://github.com/google/protobuf.git $PROTOBUF_ROOT -b '3.2.x'
cd $PROTOBUF_ROOT
./autogen.sh
./configure
make "-j$(nproc)"
sudo make install
sudo ldconfig
cd $PROTOBUF_ROOT/python
sudo python setup.py install --cpp_implementation
#
#
cd
sudo apt-get install --no-install-recommends build-essential cmake git gfortran libatlas-base-dev libboost-filesystem-dev libboost-python-dev
#
#
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libboost-all-dev libhdf5-serial-dev \
libgflags-dev libgoogle-glog-dev liblmdb-dev protobuf-compiler
#
/bin/echo -e "\e[1;32mA Reboot may be required before installing Caffe .........\e[0m"
#
#
sudo usermod -a -G video $USER
/bin/echo -e "\e[1;32mCloning Caffe into the home directory\e[0m"
# Place caffe in the home directory
cd $HOME
# Git clone Caffe
git clone https://github.com/BVLC/caffe.git 
cd caffe 
cp Makefile.config.example Makefile.config
# If cuDNN is found cmake uses it in the makefile
# Regen the makefile; On 16.04, aarch64 has issues with a static cuda runtime
cmake -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
# Include the hdf5 directory for the includes; 16.04 previously had issues for some reason
# The TX2 seems to handle this correctly now
# echo "INCLUDE_DIRS += /usr/include/hdf5/serial/" >> Makefile.config
/bin/echo -e "\e[1;32mCompiling Caffe\e[0m"
make -j6 all
# Run the tests to make sure everything works
/bin/echo -e "\e[1;32mRunning Caffe Tests\e[0m"
make -j6 runtest
# The following is a quick timing test ...
# tools/caffe time --model=models/bvlc_alexnet/deploy.prototxt --gpu=0



