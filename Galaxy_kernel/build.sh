#!/bin/bash

CROSS_COMPILE=../arm-eabi-4.4.3/bin/arm-eabi-

INITRAMFS=../initramfs

CPU_NUMBER=`grep 'processor' /proc/cpuinfo | wc -l`

TMP_INITRAMFS=/tmp/sgs4g_initramfs

BUILD_DIR="./"

##############################################################################

if [ "$1" == "-config" ] ; then
  KCONFIG=vibrantplus_antsvx_defconfig
  echo Creating $KCONFIG config...
  make $KCONFIG
  exit 0
fi

if [ "$1" == "-config2" ] ; then
  KCONFIG=vibrantplus_antsvx_noocuv_defconfig
  echo Creating $KCONFIG config...
  make $KCONFIG
  exit 0
fi


if [ "$1" == "-clean" ] ; then
  echo Cleaning project...
  rm $(find $BUILD_DIR -name '*.ko') > /dev/null 2>&1
  rm $(find $BUILD_DIR -name '*.o') > /dev/null 2>&1
  rm $(find $BUILD_DIR -name '*.bak') > /dev/null 2>&1
  rm $(find $BUILD_DIR -name '*.old') > /dev/null 2>&1
  rm $(find $BUILD_DIR -name '*.tmp') > /dev/null 2>&1
  rm -rf $TMP_INITRAMFS
  exit 0
fi

COMPILEONLY=0
PACKONLY=0

if [ "$1" == "-co" ] ; then # compile only
  echo Compilinig only, no initramfs.
  COMPILEONLY=1
fi

if [ "$1" == "-pack" ] ; then # compile only
  echo Packing zImage with initramfs.
  PACKONLY=1
  COMPILEONLY=0
fi

#

if [ ! -e ${TMP_INITRAMFS} ] ; then
  mkdir ${TMP_INITRAMFS} > /dev/null 2>&1
fi

# some cleanup

if [ $COMPILEONLY == 0 ] && [ $PACKONLY == 0 ] ; then
  echo Cleaning previous build...
  rm $(find $BUILD_DIR -name '*.ko') > /dev/null 2>&1
fi

if [ $PACKONLY == 0 ] ; then

# run the build

echo Building...

make -j$CPU_NUMBER CROSS_COMPILE=$CROSS_COMPILE

fi

#

if [ $COMPILEONLY == 0 ] ; then

  # replace kernel objects with newly build files

  echo Preparing initramfs...

  rm -rf ${TMP_INITRAMFS} > /dev/null 2>&1
  cp -rf ${INITRAMFS} ${TMP_INITRAMFS}
  find ${TMP_INITRAMFS} -name '\.git' -o -name '\.gitignore' -o -name 'EMPTY_DIRECTORY' | xargs rm -rf

  if [ $PACKONLY == 0 ] ; then

    echo Populating initramfs with updated kernel objects...

    cp $(find $BUILD_DIR -name '*.ko') ${TMP_INITRAMFS}/lib/modules/
    cp $(find $BUILD_DIR -name '*.ko') ${INITRAMFS}/lib/modules/

  fi

  rm -rf $BUILD_DIR/usr/{built-in.o,initramfs_data.{o,cpio*}} > /dev/null 2>&1
  rm $BUILD_DIR/arch/arm/boot/Image > /dev/null 2>&1
  rm $BUILD_DIR/arch/arm/boot/zImage > /dev/null 2>&1

  # run make again to build zImage with latest kernel objects

  echo Rebuilding zImage...

  make -j$CPU_NUMBER CROSS_COMPILE=$CROSS_COMPILE

fi

echo Done.
