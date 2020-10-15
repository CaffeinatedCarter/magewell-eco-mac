ifneq ($(KERNELRELEASE),)

MWEcoCapture-objs :=	  \
	common/dma/mw-dma-mem.o						\
	common/dma/mw-dma-user.o					\
	common/dma/mw-dma-kernel.o					\
	common/dma/mw-dma-phy.o						\
	common/ospi/linux-file.o  					\
	common/ospi/ospi-linux.o					\
	EcoCapture/mw-alsa.o						\
	EcoCapture/mw-v4l2.o						\
	EcoCapture/v4l2-sg-buf.o					\
	EcoCapture/pci-main.o

MWEcoCapture-objs += MWEcoCaptureLib.o

obj-m += MWEcoCapture.o

EXTRA_CFLAGS += -I$(src) -I$(src)/public -I$(src)/common
# strip debug symbols
EXTRA_LDFLAGS += -S

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

PWD := $(shell pwd)

all: MWEcoCaptureLib.o
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

OS_ARCH := $(shell uname -m | sed -e 's/i.86/i386/')
ifneq ($(OS_ARCH), i386)
	MWEcoCaptureLibFile = MWEcoCaptureLib_64.lib
else
	MWEcoCaptureLibFile = MWEcoCaptureLib_32.lib
endif
MWEcoCaptureLib.o:
	cp $(MWEcoCaptureLibFile) MWEcoCaptureLib.o

endif
