#!/bin/bash

export ARCH=arm
export CROSS_COMPILE="/home/giga/dev/Toolchains/arm-eabi-4.8/bin/arm-eabi-"


make msm_p839f30_defconfig
#make msm8939_defconfig 
make -j16

./dtbToolCM -2 -o dt.img -s 2048 -p scripts/dtc/ arch/arm/boot/dts/

#cp ./arch/arm/boot/zImage ./out/zImage

