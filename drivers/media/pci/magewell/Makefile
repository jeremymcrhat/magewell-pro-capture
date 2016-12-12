magewell-objs := core.o irq-control.o video.o ds28e01.o spi_master.o flash.o gpio.o i2c-master.o pcie-dma-controller.o v4l2-sg-buf.o pcie-dma-desc-chain.o os_support.o

MY_CFLAGS += -g -DDEBUG
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}

obj-$(CONFIG_VIDEO_MAGEWELL) += magewell.o
