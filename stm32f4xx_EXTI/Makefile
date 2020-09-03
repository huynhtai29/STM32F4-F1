
PROJ_NAME = blinky
OUTPATH = build
BINPATH :=/home/huynhtai/Downloads/TOOLCHAIN/gcc-arm-none-eabi-5_4-2016q3/bin/

#BINPATH :=/home/huynhtai/Downloads/TOOLCHAIN/gcc-arm-none-eabi-8-2019-q3-update-linux/gcc-arm-none-eabi-8-2019-q3-update/bin/
OUTPATH := $(abspath $(OUTPATH))
BASEDIR := $(abspath ./)
MKDIR_P = mkdir -
LIBPATHS = $(BASEDIR)/STM32F4xx_StdPeriph_Driver/lib
## Directory file .h and .c in Library STM32f4xx_StdPeriph_Driver
INC = $(addprefix ./,STM32F4xx_StdPeriph_Driver/inc)
SRC = $(addprefix ./,STM32F4xx_StdPeriph_Driver/src)
S_SRC = $(wildcard $(SRC)/*.c)
S_INC = $(wildcard $(INC)/*.h)
## Create Library static STM32f4xx_StdPeriph_Driver
OBJS = $(S_SRC:.c=.o)
LIB = stdperiph.a
LIB_PATH = $(LIBPATHS)/$(LIB)

## Create Libarary static FreeRTOS
RTOSPATH = $(addprefix ./,RTOS)
INC_RTOS = $(RTOSPATH)/include
INC_RTOS_ARM = $(RTOSPATH)/ARM_CM4F
RTOS_SRC = $(wildcard $(RTOSPATH)/source/*.c)\
			$(wildcard $(RTOSPATH)/ARM_CM4F/*.c)\
			$(RTOSPATH)/ARM_CM4F/MEMO/heap_4.c	\
			$(wildcard $(RTOSPATH)/ARM_CM4F/Common/*.c)		
RTOS_OBJS = $(RTOS_SRC:.c=.o)
RTOS_LIB = freertos.a
RTOS_LIB_PATH = $(LIBPATHS)/$(RTOS_LIB)
###############

##Directory file .h and .c Project
P_SRC = $(wildcard *.c)
P_SRCS = $(wildcard *.s)

AS=$(BINPATH)arm-none-eabi-as
CC=$(BINPATH)arm-none-eabi-gcc
LD=$(BINPATH)arm-none-eabi-gcc
AR=$(BINPATH)arm-none-eabi-ar
OBJCOPY=$(BINPATH)arm-none-eabi-objcopy
OBJDUMP=$(BINPATH)arm-none-eabi-objdump
SIZE=$(BINPATH)arm-none-eabi-size

##Linker
LINKER_SCRIPT = stm32_flash.ld

## Optional for compiler
CPU = -mcpu=cortex-m4 -mthumb
CFLAGS  = $(CPU) -c -std=gnu99 -g -O3 -g3 -Wall -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections -fno-strict-aliasing
LDFLAGS  = $(CPU) -mlittle-endian  -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostartfiles -Wl,--gc-sections,-Map=$(OUTPATH)/$(PROJ_NAME).map,--cref --specs=nano.specs

## Directory link Libary
INCLUDE_PATHS = -I$(INC) -I$(BASEDIR) -I$(INC_RTOS) -I$(INC_RTOS_ARM)
LIBS= -lc -lgcc -lnosys


##Compile file object in project
P_OBJS = $(P_SRC:.c=.o)
P_OBJS += $(P_SRCS:.s=.o)
.PHONY: lib proj
all: lib_rtos  lib proj
%.o : %.c    
	$(CC) $(CFLAGS) -c -o $@ $^ $(INCLUDE_PATHS)  

lib_rtos:$(RTOS_OBJS)
	$(AR) -r $(RTOS_LIB_PATH) $(RTOS_OBJS)

lib:$(OBJS)
	$(AR) -r $(LIB_PATH) $(OBJS)

proj:$(OUTPATH)/$(PROJ_NAME).elf
$(OUTPATH)/$(PROJ_NAME).elf:$(P_OBJS)
	$(CC) $(LDFLAGS) -T$(LINKER_SCRIPT) -L$(LIBPATHS) -o $@ $^ $(LIB_PATH) $(RTOS_LIB_PATH) $(LIBS) $(LD_SYS_LIBS)
	$(OBJCOPY) -O ihex $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).bin
	
clean:
	rm -r $(RTOS_OBJS)
	rm -r $(OUTPATH)/*
	rm $(OBJS)
	rm $(P_OBJS)
	rm -r $(LIB_PATH)
	rm -r $(RTOS_LIB_PATH)
	