obj-m := hrtimer.o
PWD := $(shell pwd)
KDIR := /lib/modules/$(shell uname -r)/build
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	sudo rmmod hrtimer
	make -C $(KDIR) M=$(PWD) clean
load:
	sudo insmod hrtimer.ko
unload:
	sudo rmmod hrtimer
