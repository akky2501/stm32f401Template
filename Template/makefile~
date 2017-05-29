SHELL = cmd.exe
TARGET_ARCH   = -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
INCLUDE_DIRS  = -I . \
				-I ../../Libraries \
				-I ../../Libraries/STM32F4xx_StdPeriph_Driver/inc/ \
				-I ../../Libraries/CMSIS/Device/ST/STM32F4xx/Include/ \
				-I ../../Libraries/CMSIS/Include/ \
				-I $(TOOLDIR)../arm-none-eabi/include/ \
				-I $(TOOLDIR)../arm-none-eabi/include/c++/4.9.3/
#STARTUP_DIR = ../../Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7/
STARTUP_DIR = startups/
BOARD_OPTS = -DHSE_VALUE=\(\(uint32_t\)8000000\) -DSTM32F401xx
FIRMWARE_OPTS = -DUSE_STDPERIPH_DRIVER
COMPILE_OPTS  = -std=c++11 -Os -g3 -ffunction-sections -fdata-sections -fsigned-char -fno-rtti -fno-exceptions -Wall -fmessage-length=0 $(INCLUDE_DIRS) $(BOARD_OPTS) $(FIRMWARE_OPTS)

TOOLDIR = ../../gcc-arm-none-eabi-4_9-2015q1-20150306-win32/bin/
CC      = $(TOOLDIR)arm-none-eabi-g++
CXX		= $(CC)
AS      = $(CC)
LD      = $(CC)
AR      = $(TOOLDIR)arm-none-eabi-ar
OBJCOPY = $(TOOLDIR)arm-none-eabi-objcopy
CFLAGS  = $(COMPILE_OPTS)
CXXFLAGS= $(COMPILE_OPTS)
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS)
LDFLAGS = -Wl,--gc-sections,-Map=bin\main.map,-cref -T stm32_flash.ld $(INCLUDE_DIRS) -lstdc++ -L $(TOOLDIR)../arm-none-eabi/lib/thumb -L ../../Libraries

all: libstm32f4xx startup bin\main.hex

# main.o is compiled by suffix rule automatucally
bin\main.hex: $(patsubst %.c,%.o,$(wildcard *.c)) $(patsubst %.cpp,%.o,$(wildcard *.cpp)) $(STARTUP_DIR)startup_stm32f4xx.o ../../Libraries/libstm32f4xx.a
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o bin\main.elf
	$(OBJCOPY) -O ihex bin\main.elf bin\main.hex

# many of xxx.o are compiled by suffix rule automatically
LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/src/*.c)))

libstm32f4xx: $(LIB_OBJS)
	$(AR) cr ../../Libraries/libstm32f4xx.a $(LIB_OBJS)

startup:
	$(AS) -o $(STARTUP_DIR)/startup_stm32f4xx.o $(ASFLAGS) $(STARTUP_DIR)startup_stm32f401xx.s

$(LIB_OBJS): \
 $(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/inc/*.h) \
 $(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/src/*.c) \
 makefile

clean:
	del /f /q *.o *.s bin\*
