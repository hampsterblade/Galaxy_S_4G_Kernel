#!/bin/bash

#delete old zimages
rm oczImage >/dev/null 2>&1
rm zImage >/dev/null 2>&1

#move to kernel directory
cd Galaxy_kernel

#build Overclocked kernel
echo Cleaning old build
./build.sh -clean 2>&1 >../build.log | tee ../error.log
echo Doing OC config
./build.sh -config 2>&1 >>../build.log | tee -a ../error.log
echo Building OC Kernel
./build.sh 2>&1 >>/build.log | tee -a ../error.log
cp ./arch/arm/boot/zImage ../oczImage

#build non overclocked version
echo Cleaning old build
./build.sh -clean 2>&1 >>../build.log | tee -a ../error.log
echo Doing nonOC config
./build.sh -config2 2>&1 >>../build.log | tee -a ../error.log
echo Building nonOC Kernel
./build.sh 2>&1 >>../build.log | tee -a ../error.log
cp ./arch/arm/boot/zImage ../zImage

echo build complete.  The kernel images are in your current directory.
