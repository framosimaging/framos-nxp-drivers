Supported NXP platforms:
  - i.MX 8M Plus EVK

# Cross-Compile on host system
## Procedure:
Prerequisites:
- Installed Ubuntu 20.04 OS or higher on Host System.
- Downloaded NXP documentation for [Linux version 6.6.3_1.0.0](https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX).
- Installed Yocto Toolchain 6.6.3_1.0.0 - follow the instructions in i.MX_Yocto_Project_User's_Guide document from documentation.
- Follow the chapter building and testing of [AN13712 NXP Application note](https://docs.nxp.com/bundle/AN13712/page/topics/build_and_test.html) to download and build repositories:
   - NXP kernel - linux-imx
   - isp-imx
   - isp-vvcam

1. Export full path of NXP directory where *linux-imx*, *isp-imx* and *isp-vvcam* are located:
    ```
    export NXP_DIR=<install-path>
    ```
2. Clone the Framos git repository to the Home directory on the host system:
    ```
    cd ~
    
    git clone https://github.com/framosgmbh/framos-nxp-drivers
    ```
3. Copy & replace NXP clean source files & folders with Framos modified files & folders from github
    ```
    cp -r ~/framos-nxp-drivers/linux-imx $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-imx-4.2.2.24.1 $NXP_DIR/.
    cp -r ~/framos-nxp-drivers/isp-vvcam $NXP_DIR/.
    ```
4. Build device tree:
    ```
    cd $NXP_DIR/linux-imx
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    make dtbs
    ```
5. Build isp-imx:
    ```
    cd $NXP_DIR/isp-imx-4.2.2.24.1
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    ./build-all-isp.sh
    ```
6. Build isp-vvcam:
    ```
    cd $NXP_DIR/isp-vvcam
    . /opt/fsl-imx-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux
    export KERNEL_SOURCE_DIR=$NXP_DIR/linux-imx
    ./build-all-vvcam.sh
    ```
7. Flash SD card with image from [i.MX 8M Plus EVK Image for NXP kernel Linux 6.6.3_1.0.0](https://www.nxp.com/design/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX).
   - Direct link to the image is [here](https://www.nxp.com/webapp/sps/download/license.jsp?colCode=L6.6.3_1.0.0_MX8MP&appType=file1&DOWNLOAD_ID=null), the documentation can be found at the same link.
   - For ﬂashing and booting instructions follow section 4.3 and 4.5 in Linux User Guide in the documentation. Alternatively, you can use [Balena Etcher](https://etcher.balena.io/) tool to ﬂash one of the following images:
       - imx-image-multimedia-imx8mpevk.wic
       - imx-image-full-imx8mpevk.wic
   - Set SW4 switch to 0011 on the board and connect to the platform debug port via USB to the host and then connect to target using serial port (we used Teraterm with 115200 bauds).
8. Copy needed files and folders from host to target.
