#!/bin/sh
#
# Start the isp_media_server in the configuration from user
# (c) NXP 2020-2022
# (c) Framos 2024
#

RUN_SCRIPT=`realpath -s $0`
RUN_SCRIPT_PATH=`dirname $RUN_SCRIPT`
echo "RUN_SCRIPT=$RUN_SCRIPT"
echo "RUN_SCRIPT_PATH=$RUN_SCRIPT_PATH"

LOAD_MODULES="0" # do not load modules, they are automatically loaded if present in /lib/modules
LOCAL_RUN="0" # search modules in /lib/modules and libraries in /usr/lib
RUN_OPTION=""
LOCAL_RUN="0"
# an array with the modules to load, insertion order
declare -a MODULES=("imx8-media-dev" "vvcam-dwe" "vvcam-isp" "vvcam-video")

USAGE="Usage:\n"
USAGE+="run.sh -c <isp_config> &\n"
USAGE+="Supported configurations:\n"

# parse command line arguments
while [ "$1" != "" ]; do
	case $1 in
		-c | --config )
			shift
			ISP_CONFIG=$1
			;;
		-l | --local )
			LOCAL_RUN="1"
			;;
		-lm | --load-modules )
			LOAD_MODULES="1"
			;;
		* )
			echo -e "$USAGE" >&2
			exit 1
	esac
	shift
done

write_default_mode_files () {
	# IMX662 modes file
	echo -n "" > IMX662_MODES.txt
	echo "[mode.0]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_1920x1080.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_1920x1080.json\"" >> IMX662_MODES.txt
	echo "[mode.1]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_1280x720.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_1280x720.json\"" >> IMX662_MODES.txt
	echo "[mode.2]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_960x540.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_960x540.json\"" >> IMX662_MODES.txt
	echo "[mode.3]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_640x480.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_640x480.json\"" >> IMX662_MODES.txt
	echo "[mode.4]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_HDR_1936x1100.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_1920x1080.json\"" >> IMX662_MODES.txt
	echo "[mode.5]" >> IMX662_MODES.txt
	echo "xml = \"IMX662_Basic_HDR_1936x1100.xml\"" >> IMX662_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX662_Basic_1920x1080.json\"" >> IMX662_MODES.txt
	# IMX676 modes file
	echo -n "" > IMX676_MODES.txt
	echo "[mode.0]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_3536x3072.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_3536x3072.json\"" >> IMX676_MODES.txt
	echo "[mode.1]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_3536x2140.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_3536x2140.json\"" >> IMX676_MODES.txt
	echo "[mode.2]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_1776x1778.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_1776x1778.json\"" >> IMX676_MODES.txt
	echo "[mode.3]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_1760x1070.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_1760x1070.json\"" >> IMX676_MODES.txt
	echo "[mode.4]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_HDR_3552x3072.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_3536x3072.json\"" >> IMX676_MODES.txt
	echo "[mode.5]" >> IMX676_MODES.txt
	echo "xml = \"IMX676_Basic_HDR_3552x3072.xml\"" >> IMX676_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX676_Basic_3536x3072.json\"" >> IMX676_MODES.txt
	# IMX678 modes file
	echo -n "" > IMX678_MODES.txt
	echo "[mode.0]" >> IMX678_MODES.txt
	echo "xml = \"IMX678_Basic_3840x2160.xml\"" >> IMX678_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX678_Basic_3840x2160.json\"" >> IMX678_MODES.txt
	echo "[mode.1]" >> IMX678_MODES.txt
	echo "xml = \"IMX678_Basic_1920x1080.xml\"" >> IMX678_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX678_Basic_1920x1080.json\"" >> IMX678_MODES.txt
	echo "[mode.2]" >> IMX678_MODES.txt
	echo "xml = \"IMX678_Basic_HDR_3856x2180.xml\"" >> IMX678_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX678_Basic_3840x2160.json\"" >> IMX678_MODES.txt
	echo "[mode.3]" >> IMX678_MODES.txt
	echo "xml = \"IMX678_Basic_HDR_3856x2180.xml\"" >> IMX678_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX678_Basic_3840x2160.json\"" >> IMX678_MODES.txt
	# IMX900 modes file
	echo -n "" > IMX900_MODES.txt
	echo "[mode.0]" >> IMX900_MODES.txt
	echo "xml = \"IMX900_Basic_2048x1536.xml\"" >> IMX900_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX900_Basic_2048x1536.json\"" >> IMX900_MODES.txt
	echo "[mode.1]" >> IMX900_MODES.txt
	echo "xml = \"IMX900_Basic_1920x1080.xml\"" >> IMX900_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX900_Basic_1920x1080.json\"" >> IMX900_MODES.txt
	echo "[mode.2]" >> IMX900_MODES.txt
	echo "xml = \"IMX900_Basic_1024x768.xml\"" >> IMX900_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX900_Basic_1024x768.json\"" >> IMX900_MODES.txt
	echo "[mode.3]" >> IMX900_MODES.txt
	echo "xml = \"IMX900_Basic_2048x154.xml\"" >> IMX900_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX900_Basic_2048x154.json\"" >> IMX900_MODES.txt
	echo "[mode.4]" >> IMX900_MODES.txt
	echo "xml = \"IMX900_Basic_1008x704.xml\"" >> IMX900_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_IMX900_Basic_1008x704.json\"" >> IMX900_MODES.txt
	# VB1943 modes file
	echo -n "" > VB1943_MODES.txt
	echo "[mode.0]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_2560x1984.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_2560x1984.json\"" >> VB1943_MODES.txt
	echo "[mode.1]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_2064x1552.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_2064x1552.json\"" >> VB1943_MODES.txt
	echo "[mode.2]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_1920x1080.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_1920x1080.json\"" >> VB1943_MODES.txt
	echo "[mode.3]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_2560x1984_12bit.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_2560x1984.json\"" >> VB1943_MODES.txt
	echo "[mode.4]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_2064x1552_12bit.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_2064x1552.json\"" >> VB1943_MODES.txt
	echo "[mode.5]" >> VB1943_MODES.txt
	echo "xml = \"VB1943_Basic_1920x1080_12bit.xml\"" >> VB1943_MODES.txt
	echo "dwe = \"dewarp_config/sensor_dwe_VB1943_Basic_1920x1080.json\"" >> VB1943_MODES.txt
}

# VB5943 (mono VB1943)
select_vb5943_xml () {
	local ent node csi cfg
	command -v media-ctl > /dev/null || return
	media-ctl -p 2> /dev/null | awk '
		/^- entity .*: vb1943 / { ent = 1; next }
		/^- entity / { ent = 0 }
		ent && /device node name/ { node = $NF }
		ent && /-> "mxc-mipi-csi2\.[0-9]"/ { match($0, /csi2\.[0-9]/); print node, substr($0, RSTART + 5, 1) }' |
	while read node csi; do
		cfg="Sensor${csi}_Entry.cfg"
		[ -f "$cfg" ] && grep -q "VB1943_Basic_" "$cfg" || continue
		if ! v4l2-ctl -d "$node" -L 2> /dev/null | grep -q "^[[:space:]]*1: RGB$"; then
			# only the xml lines, the dwe lines must keep the VB1943_Basic_ dewarp files
			sed -i "/^xml = /s/VB1943_Basic_/VB5943_Basic_/" "$cfg"
			echo "VB1943 on mxc-mipi-csi2.$csi is mono (VB5943), $cfg uses VB5943_Basic_*.xml"
		fi
	done
}

# write the sensonr config file
write_sensor_cfg_file () {
	local SENSOR_FILE="$1"
	local CAM_NAME="$2"
	local DRV_FILE="$3"
	local MODE_FILE="$4"
	local MODE="$5"

	echo -n "" > $SENSOR_FILE
	echo "name = \"$CAM_NAME\"" >> $SENSOR_FILE
	echo "drv = \"$DRV_FILE\"" >> $SENSOR_FILE
	echo "mode = $MODE" >> $SENSOR_FILE
	cat $MODE_FILE >> $SENSOR_FILE

	if [ ! -f $DRV_FILE ]; then
		echo "File does not exist: $DRV_FILE" >&2
		exit 1
	fi
	if [ ! -f $MODE_FILE ]; then
		echo "File does not exist: $MODE_FILE" >&2
		exit 1
	fi
}

# write the sensonr config file
set_libs_path () {
	local ONE_LIB="$1"
	LIB_PATH=`find $RUN_SCRIPT_PATH -name $ONE_LIB | head -1`
	if [ ! -f "$LIB_PATH" ]; then
		LIB_PATH=`find $RUN_SCRIPT_PATH/../../../usr -name $ONE_LIB | head -1`
		if [ ! -f "$LIB_PATH" ]; then
			echo "$ONE_LIB not found in $RUN_SCRIPT_PATH"
			echo "$ONE_LIB not found in $RUN_SCRIPT_PATH/../../../usr"
			exit 1
		fi
	fi
	LIB_PATH=`realpath $LIB_PATH`
	export LD_LIBRARY_PATH=`dirname $LIB_PATH`:/usr/lib:$LD_LIBRARY_PATH
	echo "LD_LIBRARY_PATH set to $LD_LIBRARY_PATH"
}

load_module () {
	local MODULE="$1.ko"
	local MODULE_PARAMS="$2"

	# return directly if already loaded.
	MODULENAME=`echo $1 | sed 's/-/_/g'`
	echo $MODULENAME
	if lsmod | grep "$MODULENAME" ; then
		echo "$1 already loaded."
		return 0
	fi

	if [ "$LOCAL_RUN" == "1" ]; then
		MODULE_SEARCH=$RUN_SCRIPT_PATH
		MODULE_PATH=`find $MODULE_SEARCH -name $MODULE | head -1`
		if [ "$MODULE_PATH" == "" ]; then
			MODULE_SEARCH=$RUN_SCRIPT_PATH/../../../modules
			MODULE_PATH=`find $MODULE_SEARCH -name $MODULE | head -1`
			if [ "$MODULE_PATH" == "" ]; then
				echo "Module $MODULE not found in $RUN_SCRIPT_PATH"
				echo "Module $MODULE not found in $MODULE_SEARCH"
				exit 1
			fi
		fi
		MODULE_PATH=`realpath $MODULE_PATH`
	else
		MODULE_SEARCH=/lib/modules/`uname -r`/
		MODULE_PATH=`find $MODULE_SEARCH -name $MODULE | head -1`
		if [ "$MODULE_PATH" == "" ]; then
			echo "Module $MODULE not found in $MODULE_SEARCH"
			exit 1
		fi
		MODULE_PATH=`realpath $MODULE_PATH`
	fi
	insmod $MODULE_PATH $MODULE_PARAMS  || exit 1
	echo "Loaded $MODULE_PATH $MODULE_PARAMS"
}

load_modules () {
	# remove any previous instances of the modules
	n=${#MODULES_TO_REMOVE[*]}
	for (( i = n-1; i >= 0; i-- ))
	do
		echo "Removing ${MODULES_TO_REMOVE[i]}..."
		rmmod ${MODULES_TO_REMOVE[i]} &>/dev/null
		#LSMOD_STATUS=`lsmod | grep "${MODULES[i]}"`
		#echo "LSMOD_STATUS=$LSMOD_STATUS"
		if lsmod | grep "${MODULES_TO_REMOVE[i]}" ; then
			echo "Removing ${MODULES_TO_REMOVE[i]} failed!"
			exit 1
		fi
	done

	# and now clean load the modules
	for i in "${MODULES[@]}"
	do
		echo "Loading module $i ..."
		load_module $i
	done
}

write_default_mode_files

echo "Trying configuration \"$ISP_CONFIG\"..."
case "$ISP_CONFIG" in
		imx662 )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx662" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx662" "${MODULES[@]}")
			RUN_OPTION="CAMERA0"
			CAM_NAME="imx662"
			DRV_FILE="imx662.drv"
			MODE_FILE="IMX662_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		dual_imx662 )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx662" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx662" "${MODULES[@]}")
			RUN_OPTION="DUAL_CAMERA"
			CAM_NAME="imx662"
			DRV_FILE="imx662.drv"
			MODE_FILE="IMX662_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			write_sensor_cfg_file "Sensor1_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		imx676_4k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx676" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx676" "${MODULES[@]}")
			RUN_OPTION="CAMERA0"
			CAM_NAME="imx676"
			DRV_FILE="imx676.drv"
			MODE_FILE="IMX676_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;

		dual_imx676_4k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx676" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx676" "${MODULES[@]}")
			RUN_OPTION="DUAL_CAMERA"
			CAM_NAME="imx676"
			DRV_FILE="imx676.drv"
			MODE_FILE="IMX676_MODES.txt"
			MODE="3"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			write_sensor_cfg_file "Sensor1_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		imx678_4k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx678" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx678" "${MODULES[@]}")
			RUN_OPTION="CAMERA0"
			CAM_NAME="imx678"
			DRV_FILE="imx678.drv"
			MODE_FILE="IMX678_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		dual_imx678_4k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx678" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx678" "${MODULES[@]}")
			RUN_OPTION="DUAL_CAMERA"
			CAM_NAME="imx678"
			DRV_FILE="imx678.drv"
			MODE_FILE="IMX678_MODES.txt"
			MODE="1"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			write_sensor_cfg_file "Sensor1_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		imx900_2k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx900" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx900" "${MODULES[@]}")
			RUN_OPTION="CAMERA0"
			CAM_NAME="imx900"
			DRV_FILE="imx900.drv"
			MODE_FILE="IMX900_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		dual_imx900_2k )
			MODULES_TO_REMOVE=("max96792" "max96793" "imx900" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "imx900" "${MODULES[@]}")
			RUN_OPTION="DUAL_CAMERA"
			CAM_NAME="imx900"
			DRV_FILE="imx900.drv"
			MODE_FILE="IMX900_MODES.txt"
			MODE="1"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			write_sensor_cfg_file "Sensor1_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		vb1943_2k )
			MODULES_TO_REMOVE=("max96792" "max96793" "vb1943" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "vb1943" "${MODULES[@]}")
			RUN_OPTION="CAMERA0"
			CAM_NAME="vb1943"
			DRV_FILE="vb1943.drv"
			MODE_FILE="VB1943_MODES.txt"
			MODE="0"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		dual_vb1943_2k )
			MODULES_TO_REMOVE=("max96792" "max96793" "vb1943" "${MODULES[@]}")
			MODULES=("max96792" "max96793" "vb1943" "${MODULES[@]}")
			RUN_OPTION="DUAL_CAMERA"
			CAM_NAME="vb1943"
			DRV_FILE="vb1943.drv"
			MODE_FILE="VB1943_MODES.txt"
			MODE="2"
			write_sensor_cfg_file "Sensor0_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			write_sensor_cfg_file "Sensor1_Entry.cfg" $CAM_NAME $DRV_FILE $MODE_FILE $MODE
			;;
		 *)
			echo "ISP configuration \"$ISP_CONFIG\" unsupported."
			echo -e "$USAGE" >&2
			exit 1
			;;
	esac

PIDS_TO_KILL=`pgrep -f video_test\|isp_media_server`
if [ ! -z "$PIDS_TO_KILL" ]
then
	echo "Killing preexisting instances of video_test and isp_media_server:"
	echo `ps $PIDS_TO_KILL`
	pkill -f video_test\|isp_media_server
fi

# Need a sure way to wait untill all the above processes terminated
sleep 1

if [ "$LOAD_MODULES" == "1" ]; then
	load_modules
fi

if [ "$LOCAL_RUN" == "1" ]; then
	set_libs_path "libmedia_server.so"
fi

select_vb5943_xml

echo "Starting isp_media_server with configuration file $RUN_OPTION"
./isp_media_server $RUN_OPTION

# this should now work
# gst-launch-1.0 -v v4l2src device=/dev/video0 ! "video/x-raw,format=YUY2,width=1920,height=1080" ! queue ! imxvideoconvert_g2d ! waylandsink
# gst-launch-1.0 -v v4l2src device=/dev/video0 ! "video/x-raw,format=YUY2,width=3840,height=2160" ! queue ! imxvideoconvert_g2d ! waylandsink
# gst-launch-1.0 -v v4l2src device=/dev/video0 ! waylandsink
