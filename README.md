# Framos implementation of image sensor drivers for NXP platform

This repository contains source implementation and binary package for [Framos General Optics modules FSM:GO  ](https://www.framos.com/en/fsmgo?utm_source=google&utm_medium=cpc&utm_campaign=FSM-GO_Product_Launch_2024). Here the implementation for Sony sensors imx662, imx676, imx678 and imx900 is provided with instructions on how to setup a platform, install binaries, compile sources and run a video stream on NXP platform.
Currently, only i.MX 8M Plus EVK platform is supported.

## Flashing platform

In order to install binaries one needs to prepare NXP platform. The full documentation is given at [i.MX 8M Plus EVK Image for NXP kernel Linux 6.6.3_1.0.0](https://www.nxp.com/design/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX). In order to flash SD Card:

- Download the [image](https://www.nxp.com/webapp/sps/download/license.jsp?colCode=L6.6.3_1.0.0_MX8MP&appType=file1&DOWNLOAD_ID=null).
- For ﬂashing and booting instructions follow section 4.3 and 4.5 in Linux User Guide in the [documentation](https://www.nxp.com/webapp/Download?colCode=L6.6.3_1.0.0_Docs&location=null). Alternatively, you can use [Balena Etcher](https://etcher.balena.io/) tool to ﬂash one of the following images:
	- imx-image-multimedia-imx8mpevk.wic
	- imx-image-full-imx8mpevk.wic
- Set SW4 switch to 0011 on the board and connect to the platform debug port via USB to the host and then connect to target using serial port (we used Teraterm with 115200 bauds).

## Installing binaries

If you just want to install the drivers without adapting the source code use binary packages.

- Download `Framos-iMX8MP-Driver-Binary_v*.tar.gz` file from [release page](https://github.com/framosimaging/framos-nxp-drivers/releases)
and copy it to target at `/home/root`. Extract the archive with:

```bash
tar xvzfp Framos-iMX*.tar.gz
```

- Enter package directory and run install.sh script:

```bash
cd ./package
source ./install.sh
depmod
```

- All FSM:GO sensors are installed now. To set up device tree reboot the target and press any key from terminal to enter u-boot. Then set the appropriate device tree as fdtfile (`imx8mp-evk-imx662.dtb` for single imx662 sensor, `imx8mp-evk-imx662-dual.dtb` for two sensors, `imx8mp-evk-imx662-gmsl.dtb` for singe sensor using gmsl and `imx8mp-evk-imx662-dual-gmsl.dtb` for two sensors using gmsl):

    ```bash
    setenv fdtfile imx8mp-evk-imx662.dtb
    saveenv
    boot
    ```

skip to [test streaming section](#Test-streaming-and-mode-switching-on-target) to obtain the stream.

## Compiling on host and installation on target

### Procedure

Prerequisites:

- Installed Ubuntu 20.04 OS or higher on Host System.
- Downloaded NXP documentation for [Linux version 6.6.3_1.0.0](https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX).
- Installed Yocto Toolchain 6.6.3_1.0.0 - follow the instructions in i.MX_Yocto_Project_User's_Guide document from documentation.
- Flashed SD card following [instructions](#flashing-platform)
- Download and build NXP repositories to wanted directory:
   - NXP kernel - linux-imx:

     ```bash
     git clone https://github.com/nxp-imx/linux-imx.git
     git checkout lf-6.6.3-1.0.0
     . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
     make mrproper
     make imx_v8_defconfig
     ```

   - isp-imx:

     ```bash
     wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/isp-imx-4.2.2.24.1.bin
     chmod +x isp-imx-4.2.2.24.1.bin
     ./isp-imx-4.2.2.24.1.bin
     ```

   - isp-vvcam

     ```bash
     git clone https://github.com/nxp-imx/isp-vvcam.git
     git checkout lf-6.6.3-1.0.0
     ```

1. Export full path of NXP directory where *linux-imx*, *isp-imx* and *isp-vvcam* are located and activate   toolchain

    ```bash
    export NXP_DIR=<install-path>
    export TOOLCHAIN=<toolchain-path > # /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    ```

2. Clone the Framos git repository to the Home directory on the host system:

    ```bash
    cd ~
    git clone https://github.com/framosimaging/framos-nxp-drivers
    ```

3. Copy & replace NXP clean source files & folders with Framos modified files & folders from GitHub:

    ```bash
    cp -r ~/framos-nxp-drivers/linux-imx $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-imx-4.2.2.24.1 $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-vvcam $NXP_DIR/.
    cp ~/framos-nxp-drivers/cp_to_target.sh $NXP_DIR/. # script for copying files to target
    ```

4. Activate toolchain and build kernel image:

    ```bash
    . $TOOLCHAIN
    cd $NXP_DIR/linux-imx
    make imx_v8_defconfig
    make -j$(nproc)
    ```

    Get Image tag ans save it to variable `TAG`

    ```bash
    strings ./arch/arm64/boot/Image | grep "Linux version"
    export TAG=<kernel-image-tag> # for example 6.6.3-gccf0a99701a7-dirty
    ```

5. Install kernel modules:

    ```bash
    cd $NXP_DIR/linux-imx
    mkdir ../modules
    export INSTALL_MOD_PATH=$NXP_DIR/modules
    make modules_install
    ```

6. Build isp-imx:

    ```bash
    cd $NXP_DIR/isp-imx-4.2.2.24.1
    . $TOOLCHAIN
    ./build-all-isp.sh release
    ```

7. Build isp-vvcam:

    ``` bash
    cd $NXP_DIR/isp-vvcam
    . $TOOLCHAIN
    export KERNEL_SOURCE_DIR=$NXP_DIR/linux-imx
    ./build-all-vvcam.sh release
    mkdir -p $INSTALL_MOD_PATH/lib/modules/$TAG/updates
    cp ./modules/*.ko $INSTALL_MOD_PATH/lib/modules/$TAG/updates
    ```

8. Copy needed files and folders from host to target:

    ```bash
    cd $NXP_DIR
    chmod +x cp_to_target.sh
    ./cp_to_target.sh <target-ip-address>
    ```

9. Reboot the target and press any key from terminal to enter u-boot and set the appropriate device tree as fdtﬁle (example for IMX662):

    ```bash
    setenv fdtfile imx8mp-evk-imx900.dtb
    saveenv
    boot
    ```

## Test streaming and mode switching on target

- If you are using only one sensor make sure to use CSI1 port.
- To test that everything is working as expected run:

    ```bash
    gst-launch-1.0 -v v4l2src device=/dev/video2 ! queue ! waylandsink
    ```

- If you are using dual mode you can use commands:

    ```bash
    gst-launch-1.0 -v v4l2src device=/dev/video2 ! queue ! waylandsink
    gst-launch-1.0 -v v4l2src device=/dev/video3 ! queue ! waylandsink
    ```

to visualize streams separately. To get the stream side by side use

  ```bash
  gst-launch-1.0 imxcompositor_g2d name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=960 sink_0::height=1080 sink_1::xpos=960 sink_1::ypos=0 sink_1::width=960 sink_1::height=1080 ! video/x-raw! waylandsink v4l2src device=/dev/video2 ! video/x-raw,format=YUY2,framerate=30/1 ! comp.sink_0 v4l2src device=/dev/video3 ! video/x-raw,format=YUY2,framerate=30/1 ! comp.sink_1
  ```

- To switch the streaming mode edit the isp mode conﬁguration */opt/imx8-isp/bin/run.sh* under desired conﬁguration (single or dual) by setting MODE = "<MOD_NUM>"

- Restart the isp server with:

  ```bash
  /opt/imx8-isp/bin/start_isp.sh
  ```

- To get the raw stream use

  ```bash
  v4l2-ctl -d2 --set-fmt-video=pixelformat=RG12 --stream-mmap
  ```

- wayland is limited to the maximum of 60 fps, to get the stream at maximal frame rate while using ISP functions (auto exposure, etc.) use

  ```bash
  v4l2-ctl -d2 --set-fmt-video=pixelformat=YUYV --stream-mmap
  ```
