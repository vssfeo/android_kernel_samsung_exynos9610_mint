# Makefile для внешних модулей ядра LOGGER (Helideck).
#
# Сборка выполняется ВНУТРИ дерева исходников Mint (тег xbeta-1413_a50dx),
# потому что внешний модуль линкуется к заголовкам и генерируемым файлам
# этого дерева (включая Module.symvers с kallsyms_lookup_name).
#
# Пример (вызывать из корня дерева ядра):
#   make ARCH=arm64 M=/путь/до/hdk-module modules \
#        LOCALVERSION=" - Mint Beta 1413" CC=clang HOSTCC=clang HOSTCXX=clang++ \
#        AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip

# Список модулей можно сузить переменной HDK_MODULES.
#
# Зачем: hdk_min — РЕШАЮЩИЙ модуль для бисекции ENOEXEC (у него нет ни одного
# внешнего символа). Он не должен зависеть от успеха сборки hdk_sensor_core,
# поэтому по умолчанию собираются все три, но можно собрать только нужные:
#   make M=... modules HDK_MODULES=hdk_min
HDK_MODULES ?= hdk_min hdk_probe hdk_sensor_core

obj-m += $(addsuffix .o,$(HDK_MODULES))

# При обычной сборке из корня ядра KDIR задан автоматически.
# Для standalone-сборки можно указать KDIR=<путь к дереву ядра>.
KDIR ?= $(PWD)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install