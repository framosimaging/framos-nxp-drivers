#!/bin/sh

if [ "$1" != "" ]; then
    #echo "Arg 1 is BUILD_TYPE: Release or Debug ->"
    #echo $1
    BUILD_TYPE=$1
else
    BUILD_TYPE=Release
fi

BUILD_TYPE=`echo $BUILD_TYPE| tr '[:upper:]' '[:lower:]'`

echo "BUILD_TYPE: $BUILD_TYPE"

if [ "$2" != "" ]; then
    #echo "Arg 2 is BUILD: partial, full -> "
    #echo $2
    BUILD=$2
else
    BUILD=partial
fi

echo "BUILD: $BUILD"

#BUILD_TYPE=Debug
#BUILD_TYPE=Release
#BUILD=full
#BUILD=partial


rm -rf appshell/build
mkdir -p appshell/build
cd appshell/build

if [ "$BUILD_TYPE" = 'release' ]
then
    echo "Release build --------------------->"
else
    echo "Debug build   --------------------->"
    BUILD_TYPE=debug
fi

CMAKE_INSTALL_LIBDIR=/usr/lib

if [ "$BUILD" = 'full' ]
then
echo "Full build --------------------->"

    echo "V4L2 mode build --------------------->"
    cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DISP_VERSION=ISP8000NANO_V1802 -DPLATFORM=ARM64 -DTUNINGEXT=1\
	-DQTLESS=1 -DFULL_SRC_COMPILE=1 -DWITH_DWE=1 -DWITH_DRM=1 -DSUBDEV_V4L2=1 \
	-DIMX_G2D=ON -DSDKTARGETSYSROOT=$SDKTARGETSYSROOT -DCMAKE_INSTALL_LIBDIR=$CMAKE_INSTALL_LIBDIR \
	-Werror=date-time -Wno-dev ..
else 
echo "Partial build --------------------->"

    echo "V4L2 mode build --------------------->"
	cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DISP_VERSION=ISP8000NANO_V1802 -DPLATFORM=ARM64 -DTUNINGEXT=1\
	-DQTLESS=1 -DFULL_SRC_COMPILE=1 -DWITH_DWE=1 -DWITH_DRM=1 -DSUBDEV_V4L2=1 \
	-DPARTITION_BUILD=0 -D3A_SRC_BUILD=0 -DIMX_G2D=ON -DSDKTARGETSYSROOT=$SDKTARGETSYSROOT \
	-DCMAKE_INSTALL_LIBDIR=$CMAKE_INSTALL_LIBDIR -Wno-dev ..

fi

make -j4
cd -

BUILD_OUTPUT_DIR=build_output_${BUILD_TYPE}_${BUILD}

rm -rfv  ${BUILD_OUTPUT_DIR}.tgz ${BUILD_OUTPUT_DIR}

ISP_LIB_DIR=${BUILD_OUTPUT_DIR}/usr/lib
ISP_BIN_DIR=${BUILD_OUTPUT_DIR}/opt/imx8-isp/bin
ISP_BOOT_DIR=${BUILD_OUTPUT_DIR}/boot
ISP_BLOB_DIR=${BUILD_OUTPUT_DIR}/blob

mkdir -p ${ISP_LIB_DIR}
mkdir -p ${ISP_BIN_DIR}
mkdir -p ${ISP_BOOT_DIR}
mkdir -p ${ISP_BLOB_DIR}

if [ "$BUILD_TYPE" = 'release' ]
then
BUILD_DIR=release
else
BUILD_DIR=debug
fi

cp -a appshell/build/generated/${BUILD_DIR}/lib/*.so* ${ISP_LIB_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/IMX662* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/imx662* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/IMX676* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/imx676* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/IMX678* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/imx678* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/IMX900* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/imx900* ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/isp_media_server ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/*_test ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_DIR}/bin/*.cfg ${ISP_BIN_DIR}/
cp -a appshell/build/generated/${BUILD_DIR}/bin/*.cfg ${ISP_BIN_DIR}/
cp -a appshell/build/generated/${BUILD_TYPE}/bin/tuningext ${ISP_BIN_DIR}
cp -a appshell/build/generated/${BUILD_TYPE}/bin/vvext ${ISP_BIN_DIR}

#cp -ra mediacontrol/case/ ${ISP_BIN_DIR}
cp -ra dewarp/dewarp_config/ ${ISP_BIN_DIR}
#cp utils3rd/tools/ISP8000NANO_V1802/*  ${ISP_BIN_DIR}
#cp Readme.txt ${ISP_BIN_DIR}

#cp utils3rd/tools/ISP8000NANO_V1802/run.sh  ${ISP_BIN_DIR}/run-vsi.sh
cp imx/run.sh ${ISP_BIN_DIR}/run.sh
cp imx/start_isp.sh ${ISP_BIN_DIR}

chmod +x ${ISP_BIN_DIR}/run.sh
#chmod +x ${ISP_BIN_DIR}/run-imx.sh
#chmod +x ${ISP_BIN_DIR}/memtool

cp -a ${ISP_BIN_DIR}/* ${ISP_BLOB_DIR}
cp -a ${ISP_LIB_DIR}/* ${ISP_BLOB_DIR}

#cd ..
tar zcvf ${BUILD_OUTPUT_DIR}.tgz ${BUILD_OUTPUT_DIR}
#cd 


