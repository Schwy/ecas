#generate .o
obj-m	   := squelette_rtai_nico_singletask.o

PATH := /opt/x86_64/gcc/gcc-3.4.4/bin:$(PATH)
KIT = arcom19.esiee.fr

CURRENT = 2.6.16.14-arcom1
KDIR = /usr/lib/arcom/lib/modules/${CURRENT}/build
PWD = $(shell pwd)
EXTRA_CFLAGS += -I/usr/lib/arcom/realtime/include -Wall -ffast-math -mhard-float
default:
	@echo --------------------------------------
	@echo PWD = $(PWD)
	@echo CURRENT = $(CURRENT)
	@echo KDIR = $(KDIR)
	i386 -B --uname-2.6 make -C $(KDIR) M=$(PWD) modules

clean:
	-rm -f *.o squelette*.ko .*.cmd .*.flags *.mod.c *~
	-rm -r .tmp*
send:
	scp *.ko runarcom arcom@$(KIT):/home/arcom
	ssh arcom@$(KIT)

a:
	-rm -f *.o squelette*.ko .*.cmd .*.flags *.mod.c *~
	-rm -r .tmp*
	@echo Cleaned
	@echo

	@echo PWD = $(PWD)
	@echo CURRENT = $(CURRENT)
	@echo KDIR = $(KDIR)
	i386 -B --uname-2.6 make -C $(KDIR) M=$(PWD) modules
	@echo Built
	@echo
	
	
	scp *.ko runarcom arcom@$(KIT):/home/arcom
	@echo
	ssh arcom@$(KIT)

b:
	-rm -f *.o squelette*.ko .*.cmd .*.flags *.mod.c *~
	-rm -r .tmp*
	@echo
	@echo PWD = $(PWD)
	@echo CURRENT = $(CURRENT)
	@echo KDIR = $(KDIR)
	i386 -B --uname-2.6 make -C $(KDIR) M=$(PWD) modules
	@echo
	
	
s:
	scp *.ko runarcom arcom@$(KIT):/home/arcom
	@echo
	ssh arcom@$(KIT)

c:
	ssh arcom@$(KIT)
