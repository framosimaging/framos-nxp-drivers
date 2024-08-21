#!/bin/sh

if [ -z "${KERNEL_SOURCE_DIR}" ]; then
export KERNEL_SOURCE_DIR=/build/users/$USER/proj/imx8/linux-imx
fi

#manually clean vvcam (if needed)

cd vvcam
find -name *.o | xargs rm -fv
find -name *.ko | xargs rm -fv
find -name *.o.cmd | xargs rm -fv 
cd -

cd vvcam/v4l2

#make -f 1802_chip.mk clean
make KERNEL_SRC=$KERNEL_SOURCE_DIR clean
echo "v4l2 mode build --------------------->"
make KERNEL_SRC=$KERNEL_SOURCE_DIR

cd -

rm -rf modules
mkdir -p modules

cp vvcam/v4l2/dwe/vvcam-dwe.ko modules
cp vvcam/v4l2/sensor/imx662/imx662.ko modules
cp vvcam/v4l2/sensor/imx676/imx676.ko modules
cp vvcam/v4l2/sensor/imx678/imx678.ko modules
#cp vvcam/v4l2/csi/samsung/vvcam-csis.ko modules
cp vvcam/v4l2/isp/vvcam-isp.ko modules
cp vvcam/v4l2/video/vvcam-video.ko modules
