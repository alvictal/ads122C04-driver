KDIR := ~/linux
PWD := $(shell pwd)
obj-m += ads122c04.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean