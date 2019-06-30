#!/bin/bash

export WD=~/Development
export ARCH=arm
export CROSS_COMPILE=/opt/Xilinx/SDK/2017.4/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-
export KDIR=$WD/linux-xlnx/
export PATH=$WD/u-boot-xlnx/tools/:$PATH
export PATH=$WD/linux-xlnx/scripts/dtc/:$PATH

#vivado installation
export PATH=/opt/Xilinx/Vivado/2016.2/bin/:$PATH
export PATH=/opt/Xilinx/SDK/2016.2/bin/:$PATH

export XILINXD_LICENSE_FILE=27000@license3.uni-paderborn.de

source /opt/Xilinx/Vivado/2016.2/settings64.sh
source /opt/Xilinx/SDK/2016.2/settings64.sh

#reconos settings
source $WD/reconos/tools/settings.sh



