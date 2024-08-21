#!/bin/bash

# Check if the number of arguments is correct
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <target ip address>"
  exit 1
fi

TARGET_IP="$1"

echo "Installing Framos modules to i.MX8MP board."

DEVICE_TREE_DIR="/run/media/boot-mmcblk1p1"
MODULES_DIR="/lib/modules/6.6.3-lts-next-gccf0a99701a7/updates"
OPT_BIN="/opt/imx8-isp"
USR_LIB="/usr"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

echo "Copying device trees..."
scp ./linux-imx/arch/arm64/boot/dts/freescale/imx8mp-evk-imx*.dtb root@$TARGET_IP:$DEVICE_TREE_DIR

echo "Copying vvcam sensor modules..."
scp ./isp-vvcam/vvcam/v4l2/dwe/vvcam-dwe.ko root@$TARGET_IP:$MODULES_DIR
scp ./isp-vvcam/vvcam/v4l2/isp/vvcam-isp.ko root@$TARGET_IP:$MODULES_DIR
scp ./isp-vvcam/vvcam/v4l2/video/vvcam-video.ko root@$TARGET_IP:$MODULES_DIR
scp ./isp-vvcam/vvcam/v4l2/sensor/imx662/imx662.ko root@$TARGET_IP:$MODULES_DIR
scp ./isp-vvcam/vvcam/v4l2/sensor/imx676/imx676.ko root@$TARGET_IP:$MODULES_DIR
scp ./isp-vvcam/vvcam/v4l2/sensor/imx678/imx678.ko root@$TARGET_IP:$MODULES_DIR

echo "Copying isp media server and isi drivers..."
echo "Copying /opt/bin folder to ${OPT_BIN}"
scp -r ./isp-imx-4.2.2.24.1/build_output_release_partial/opt/imx8-isp/bin root@$TARGET_IP:$OPT_BIN

echo "Copying isp libraries..."
scp -r ./isp-imx-4.2.2.24.1/build_output_release_partial/usr/lib root@$TARGET_IP:$USR_LIB

echo "Installation succesful."
