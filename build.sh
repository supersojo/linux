#!/bin/bash

# exit on error
set -e


KERNEL_SRC=$(pwd)

OUTPUT=$KERNEL_SRC/../linux-output

export ARCH=arm
export CROSS_COMPILE=/opt/gcc10-arm/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-
make O=$OUTPUT meson3_defconfig
make O=$OUTPUT
