Supported NXP platforms:
  - i.MX 8M Plus EVK

# Compiling on host and installation on target
## Procedure:
Prerequisites:
- Installed Ubuntu 20.04 OS or higher on Host System.
- Downloaded NXP documentation for [Linux version 6.6.3_1.0.0](https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX).
- Installed Yocto Toolchain 6.6.3_1.0.0 - follow the instructions in i.MX_Yocto_Project_User's_Guide document from documentation.
- Download and build NXP repositories to wanted directory:
   - NXP kernel - linux-imx:
     ```
     git clone https://github.com/nxp-imx/linux-imx.git
     git checkout lf-6.6.3-1.0.0
     . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
     make mrproper
     make imx_v8_defconfig
     make ARCH=arm64 -j8  # change num of processors 
     ```
   - isp-imx:
     ```
     wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/isp-imx-4.2.2.24.1.bin
     chmod +x isp-imx-4.2.2.24.1.bin
     ./isp-imx-4.2.2.24.1.bin
     ```
   - isp-vvcam
     ```
     git clone https://github.com/nxp-imx/isp-vvcam.git
     git checkout lf-6.6.3-1.0.0
     ```

1. Export full path of NXP directory where *linux-imx*, *isp-imx* and *isp-vvcam* are located:
    ```
    export NXP_DIR=<install-path>
    ```
2. Clone the Framos git repository to the Home directory on the host system:
    ```
    cd ~
    
    git clone https://github.com/framosgmbh/framos-nxp-drivers
    ```
3. Copy & replace NXP clean source files & folders with Framos modified files & folders from github:
    ```
    cp -r ~/framos-nxp-drivers/linux-imx $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-imx-4.2.2.24.1 $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-vvcam $NXP_DIR/.
    ```
4. Copy script for deploying files to target:
    ```
    cp ~/framos-nxp-drivers/cp_to_target.sh $NXP_DIR/.
    ```
5. Build device tree:
    ```
    cd $NXP_DIR/linux-imx
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    make dtbs
    ```
6. Build isp-imx:
    ```
    cd $NXP_DIR/isp-imx-4.2.2.24.1
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    ./build-all-isp.sh
    ```
7. Build isp-vvcam:
    ```
    cd $NXP_DIR/isp-vvcam
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    export KERNEL_SOURCE_DIR=$NXP_DIR/linux-imx
    ./build-all-vvcam.sh
    ```
8. Flash SD card with image from [i.MX 8M Plus EVK Image for NXP kernel Linux 6.6.3_1.0.0](https://www.nxp.com/design/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX).
   - Direct link to the image is [here](https://www.nxp.com/webapp/sps/download/license.jsp?colCode=L6.6.3_1.0.0_MX8MP&appType=file1&DOWNLOAD_ID=null), the documentation can be found at the same link.
   - For ﬂashing and booting instructions follow section 4.3 and 4.5 in Linux User Guide in the documentation. Alternatively, you can use [Balena Etcher](https://etcher.balena.io/) tool to ﬂash one of the following images:
       - imx-image-multimedia-imx8mpevk.wic
       - imx-image-full-imx8mpevk.wic
   - Set SW4 switch to 0011 on the board and connect to the platform debug port via USB to the host and then connect to target using serial port (we used Teraterm with 115200 bauds).
9. Copy needed files and folders from host to target:
    ```
    cd $NXP_DIR
    chmod +x cp_to_target.sh
    ./cp_to_target.sh <target-ip-address>
    ```
10. Reboot the target and press any key from terminal to enter u-boot and set the appropriate device tree as fdtﬁle (example for IMX662):
    ```
    setenv fdtfile imx8mp-evk-imx662.dtb
    saveenv
    boot
    ```

# Test streaming and mode switching on target
- If you are using only one sensor make sure to use CSI1 port.
- To test that everything is working as expected run:
  ```
  gst-launch-1.0 -v v4l2src device=/dev/video2 ! waylandsink
  ```
- If you are using dual mode you can use command:
  ```
  gst-launch-1.0 imxcompositor_g2d name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=960 sink_0::height=1080 sink_1::xpos=960 sink_1::ypos=0 sink_1::width=960 sink_1::height=1080 ! video/x-raw! waylandsink v4l2src device=/dev/video2 ! video/x-raw,format=YUY2,framerate=30/1 ! comp.sink_0 v4l2src device=/dev/video3 ! video/x-raw,format=YUY2,framerate=30/1 ! comp.sink_1
  ```
- To switch the streaming mode edit the isp mode conﬁguration */opt/imx8-isp/bin/run.sh* under desired conﬁguration (single or dual) by setting MODE = "<MOD_NUM>"
- Restart the isp server with:
  ```
  /opt/imx8-isp/bin/start_isp.sh
  ```
