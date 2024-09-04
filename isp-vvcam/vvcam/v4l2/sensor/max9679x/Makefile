PWD := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

obj-m +=max96792.o
obj-m +=max96793.o

# ccflags-y += -I$(PWD)/../../../common/
ccflags-y += -O2 -Werror

ARCH_TYPE ?= arm64
ANDROID ?= no

ifeq ($(ANDROID), yes)

V := 1

all:
	@$(MAKE) V=$(V) -C $(KERNEL_SRC) ARCH=$(ARCH_TYPE) M=$(PWD) modules
modules_install:
	@$(MAKE) V=$(V) -C $(KERNEL_SRC) M=$(PWD) modules_install
clean:
	@rm -rf modules.order Module.symvers
	@find ../ -name "*.o" | xargs rm -f
	@find ../ -name "*.ko" | xargs rm -f


else

all:
	make -C $(KERNEL_SRC) ARCH=$(ARCH_TYPE) M=$(PWD) modules
modules_install:
	make -C $(KERNEL_SRC) M=$(PWD) modules_install
clean:
	rm -rf $($(TARGET)-objs)
	make -C $(KERNEL_SRC) M=$(PWD) clean
endif
