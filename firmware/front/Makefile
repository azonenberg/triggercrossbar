SRCDIR=$(CURDIR)
ROOTDIR=$(CURDIR)/../../
CXXFLAGS=-g -O3 --std=c++17 -mcpu=cortex-m4 -DSTM32L431 -fno-exceptions -fno-rtti -g --specs=nano.specs -Wall -Wextra \
	-I$(ROOTDIR) \
	-I$(SRCDIR) \
	-I$(ROOTDIR)/stm32-cpp/devices/inc \
	-I$(ROOTDIR)/stm32-cpp/src \
	-I$(ROOTDIR)/common-embedded-platform/ \
	-DMICROKVS_WRITE_BLOCK_SIZE=8 \
	-DNO_CLEAR_SCREEN \
	-DKVS_NAMELEN=32

LDFLAGS=-Wl,-T $(ROOTDIR)/stm32-cpp/devices/link/stm32l431_32kboot_app.ld \
	 -L$(ROOTDIR)/stm32-cpp/devices/link/

CXX=arm-none-eabi-g++

.ONESHELL:
.SILENT: all
all:
	mkdir -p build
	cd build
	$(CXX) $(ROOTDIR)/stm32-cpp/src/cpu/*.* -c $(CXXFLAGS) || exit 1
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/*.cpp || exit 1
	$(CXX) $(CXXFLAGS) -c $(ROOTDIR)/microkvs/kvs/*.cpp || exit 1
	$(CXX) $(CXXFLAGS) -c $(ROOTDIR)/microkvs/driver/STM32StorageBank.cpp || exit 1
	$(CXX) $(ROOTDIR)/common-embedded-platform/core/*.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(ROOTDIR)/firmware/bsp-front/*.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(ROOTDIR)/stm32-cpp/src/newlib-stubs/*.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(ROOTDIR)/stm32-cpp/src/peripheral/*.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(ROOTDIR)/stm32-cpp/src/util/*.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(ROOTDIR)/stm32-cpp/devices/src/stm32l431.cpp -c $(CXXFLAGS) || exit 1
	$(CXX) $(CXXFLAGS) *.o $(LDFLAGS) -o ../firmware.elf
	cd ..
	arm-none-eabi-objcopy -O binary --only-section=.text --only-section=.data firmware.elf firmware.bin
	./imagesize.sh
	arm-none-eabi-strip -s -o front-stripped.elf firmware.elf

.PHONY: clean
clean:
	rm build/*.o
