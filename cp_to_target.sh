#!/bin/bash

# Check if the number of arguments is correct
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <target ip address>"
  exit 1
fi

TARGET_IP="$1"

echo "Installing Framos modules to i.MX8MP board."

DEVICE_TREE_DIR="/run/media/boot-mmcblk1p1"
MODULES_DIR="/lib/modules/$TAG"
OPT_BIN="/opt/imx8-isp"
USR_LIB="/usr"

echo "Copying Image and device trees..."
scp $NXP_DIR/linux-imx/arch/arm64/boot/Image root@$TARGET_IP:/run/media/boot-mmcblk1p1
scp $NXP_DIR/linux-imx/arch/arm64/boot/dts/freescale/imx8mp-evk-imx*.dtb root@$TARGET_IP:$DEVICE_TREE_DIR

echo "Copying kernel modules..."
ssh root@$TARGET_IP "mkdir -p /lib/modules/$TAG"
scp -r $NXP_DIR/modules/lib/modules/$TAG/kernel root@$TARGET_IP:$MODULES_DIR
scp -r $NXP_DIR/modules/lib/modules/$TAG/updates root@$TARGET_IP:$MODULES_DIR
scp $NXP_DIR/modules/lib/modules/$TAG/modules* root@$TARGET_IP:$MODULES_DIR

echo "Copying isp media server and isi drivers..."
echo "Copying /opt/bin folder to ${OPT_BIN}"
scp -r $NXP_DIR/isp-imx-4.2.2.24.1/build_output_release_partial/opt/imx8-isp/bin root@$TARGET_IP:$OPT_BIN

echo "Copying isp libraries..."
scp -r $NXP_DIR/isp-imx-4.2.2.24.1/build_output_release_partial/usr/lib root@$TARGET_IP:$USR_LIB

echo "Installation succesful."
