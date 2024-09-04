#the device/fsl/common/build/kernel.mk should be included before this file

KERNEL_SRC := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
TARGET_ARCH := $(TARGET_KERNEL_ARCH)
VVCAM_CROSS_COMPILE := aarch64-linux-gnu-

VVCAM_SRC_PATH := $(VVCAM_PATH)/vvcam/v4l2
VVCAM_OUT := $(TARGET_OUT_INTERMEDIATES)/VVCAM_OBJ

KERNEL_CFLAGS ?= KCFLAGS=-mno-android
ARCH_TYPE ?= $(TARGET_ARCH)

VVCAM_KERNELENVSH := $(VVCAM_OUT)/kernelenv.sh
.PHONY: $(VVCAM_KERNELENVSH)
$(VVCAM_KERNELENVSH):
	mkdir -p $(VVCAM_OUT)
	echo 'export KERNEL_SRC=$(KERNEL_SRC)' > $(VVCAM_KERNELENVSH)
	echo 'export CROSS_COMPILE=$(VVCAM_CROSS_COMPILE)' >> $(VVCAM_KERNELENVSH)
	echo 'export ARCH_TYPE=$(ARCH_TYPE)' >> $(VVCAM_KERNELENVSH)

vvcam: $(VVCAM_KERNELENVSH) $(VVCAM_SRC_PATH)
	$(hide) if [ ${clean_build} = 1 ]; then \
		PATH=$$PATH $(MAKE) -C $(VVCAM_SRC_PATH) ANDROID=yes clean; \
	fi
	@ . $(VVCAM_KERNELENVSH); $(kernel_build_shell_env) \
	$(MAKE) -C $(VVCAM_SRC_PATH) ANDROID=yes \
		$(CLANG_TO_COMPILE) \
		$(KERNEL_CFLAGS) \
		ARCH_TYPE=$(ARCH_TYPE) \
		DEBUG=$(DEBUG);
	cp $(VVCAM_SRC_PATH)/sensor/ov2775/ov2775.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/sensor/camera-proxy-driver/basler-camera-driver-vvcam.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/sensor/os08a20/os08a20.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/sensor/max9679x/max96792.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/sensor/max9679x/max96793.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/video/vvcam-video.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/isp/vvcam-isp.ko $(VVCAM_OUT);
	cp $(VVCAM_SRC_PATH)/dwe/vvcam-dwe.ko $(VVCAM_OUT);
