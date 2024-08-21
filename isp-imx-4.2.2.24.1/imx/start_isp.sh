#!/bin/sh
#
# Start the isp_media_server in the configuration for Framos IMX662, IMX676 or IMX678
#
# (c) Framos 2024
# (c) NXP 2020-2022
#

RUNTIME_DIR="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
NR_DEVICE_TREE_IMX662=$(grep imx662 /sys/firmware/devicetree/base/soc@0/*/i2c@*/*/*/*/compatible -l | wc -l 2> /dev/null)
NR_DEVICE_TREE_IMX676=$(grep imx676 /sys/firmware/devicetree/base/soc@0/*/i2c@*/*/*/*/compatible -l | wc -l 2> /dev/null)
NR_DEVICE_TREE_IMX678=$(grep imx678 /sys/firmware/devicetree/base/soc@0/*/i2c@*/*/*/*/compatible -l | wc -l 2> /dev/null)

# check for imx662 devices
if [ $NR_DEVICE_TREE_IMX662 -eq 2 ]; then

	echo "Starting isp_media_server for Dual IMX662"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c dual_imx662 -lm
	
elif [ $NR_DEVICE_TREE_IMX662 -eq 1 ]; then

	echo "Starting isp_media_server for IMX662"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c imx662 -lm

# check for imx676 devices
elif [ $NR_DEVICE_TREE_IMX676 -eq 2 ]; then

	echo "Starting isp_media_server for Dual IMX676"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c dual_imx676_4k -lm
	
elif [ $NR_DEVICE_TREE_IMX676 -eq 1 ]; then

	echo "Starting isp_media_server for IMX676"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c imx676_4k -lm

# check for imx678 devices
elif [ $NR_DEVICE_TREE_IMX678 -eq 2 ]; then

	echo "Starting isp_media_server for Dual IMX678"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c dual_imx678_4k -lm
	
elif [ $NR_DEVICE_TREE_IMX678 -eq 1 ]; then

	echo "Starting isp_media_server for IMX678"

	cd $RUNTIME_DIR
	
	exec ./run.sh -c imx678_4k -lm

else
	# no device tree found exit with code no device or address
	echo "No device tree found for IMX662, IMX676 or IMX678, check dtb file!" >&2
	exit 6
fi
