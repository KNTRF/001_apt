SHELL = sh
TARGET_ARCH   = -mcpu=cortex-m3 -mthumb
INCLUDE_DIRS  = -I ../../libraries \
				-I ../../libraries/STM32F10x_StdPeriph_Driver/inc \
				-I ../../libraries/CMSIS/Device/ST/STM32F10x \
				-I ../../libraries/CMSIS/Include \
				-I ../../libraries/util \
				-I arm-none-eabi/include \
				-I arm-none-eabi/include/c++/4.7.4
STARTUP_DIR = ../../libraries/CMSIS/Device/ST/STM32F10x/startup/gcc_ride7/
BOARD_OPTS = -DHSE_VALUE=12000000 -DSTM32F10X_MD \
             -DSYSCLK_FREQ_72MHz=72000000
FIRMWARE_OPTS = -DUSE_STDPERIPH_DRIVER
COMPILE_OPTS  = -Os -fsigned-char -fno-rtti -fno-exceptions -Wall -fmessage-length=0 $(INCLUDE_DIRS) $(BOARD_OPTS) $(FIRMWARE_OPTS)

CC      = arm-none-eabi-g++
CXX		= $(CC)
AS      = $(CC)
LD      = $(CC)
AR      = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = $(COMPILE_OPTS)
CXXFLAGS= $(COMPILE_OPTS)
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS) 
LDFLAGS = -Wl,--gc-sections,-Map=bin/main.map,-cref -T 128_20_dfu.ld $(INCLUDE_DIRS) -lstdc++ -L arm-none-eabi/lib/thumb -L ../../libraries

all: libstm32f10x startup bin/main.hex bin/main.bin

# main.o is compiled by suffix rule automatucally
bin/main.hex: $(patsubst %.c,%.o,$(wildcard *.c)) $(patsubst %.cpp,%.o,$(wildcard *.cpp)) $(STARTUP_DIR)startup_stm32f10x_hd.o ../../libraries/libstm32f10x.a
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o bin/main.elf 
	$(OBJCOPY) -O ihex bin/main.elf bin/main.hex

# many of xxx.o are compiled by suffix rule automatically
LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard ../../libraries/STM32F10x_StdPeriph_Driver/src/*.c)) \
 $(patsubst %.c,%.o,$(wildcard ../../libraries/CMSIS/Device/ST/STM32F10x/*.c))) \
 $(patsubst %.c,%.o,$(wildcard ../../libraries/util/*.c)) \
 $(patsubst %.cpp,%.o,$(wildcard ../../libraries/util/*.cpp))

libstm32f10x: $(LIB_OBJS)
	$(AR) cr ../../libraries/libstm32f10x.a $(LIB_OBJS)
	
startup:
	$(AS) -o $(STARTUP_DIR)/startup_stm32f10x_hd.o $(ASFLAGS) $(STARTUP_DIR)startup_stm32f10x_hd.s
 	
$(LIB_OBJS): \
 $(wildcard ../../libraries/STM32F10x_StdPeriph_Driver/inc/*.h) \
 $(wildcard ../../libraries/STM32F10x_StdPeriph_Driver/src/*.c) \
 $(wildcard ../../libraries/CMSIS/Device/ST/STM32F10x/*.h) \
 $(wildcard ../../libraries/CMSIS/Device/ST/STM32F10x/*.c) \
 $(wildcard ../../libraries/CMSIS/util/*.h) \
 $(wildcard ../../libraries/CMSIS/util/*.c) \
 $(wildcard ../../libraries/CMSIS/util/*.cpp) \
 Makefile

#-----------------------------------------------------------------------------------------
# fukuda added
# convert
bin/main.bin: 
	$(OBJCOPY) -O binary bin/main.elf bin/main.bin

flash:
	dfu-util -a0 0x0483:0xdf11 -s 0x08003000 -D bin/main.bin -d 0x0483
#------------------------------------------------------------------------------------------

clean:
	rm -f *.o *.s bin/*