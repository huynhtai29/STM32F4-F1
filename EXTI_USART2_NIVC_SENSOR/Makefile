

PROJ_NAME = blinky
OUTPATH = build
BINPATH :=/home/huynhtai/Downloads/TOOLCHAIN/gcc-arm-none-eabi-5_4-2016q3/bin/
OUTPATH := $(abspath $(OUTPATH))
BASEDIR := $(abspath ./)
MKDIR_P = mkdir -
LIBPATHS = -L$(BASEDIR)

INC = $(addprefix ./,inc)
SRC = $(addprefix ./,src)
S_SRC = $(wildcard $(SRC)/*.c)
S_INC = $(wildcard $(INC)/*.h)
P_SRC = $(wildcard *.c)
P_SRCS = $(wildcard *.s)

AS=$(BINPATH)arm-none-eabi-as
CC=$(BINPATH)arm-none-eabi-gcc
LD=$(BINPATH)arm-none-eabi-gcc
AR=$(BINPATH)/arm-none-eabi-ar
OBJCOPY=$(BINPATH)arm-none-eabi-objcopy
OBJDUMP=$(BINPATH)arm-none-eabi-objdump
SIZE=$(BINPATH)arm-none-eabi-size

LINKER_SCRIPT = stm32_flash.ld

CPU = -mcpu=cortex-m4 -mthumb
CFLAGS  = $(CPU) -c -std=gnu99 -g -O2 -Wall
LDFLAGS  = $(CPU) -mlittle-endian -mthumb-interwork -nostartfiles -Wl,--gc-sections,-Map=$(OUTPATH)/$(PROJ_NAME).map,--cref --specs=nano.specs

INCLUDE_PATHS = -I$(BASEDIR)/inc -I$(BASEDIR)
OBJS = $(S_SRC:.c=.o)
P_OBJS = $(P_SRC:.c=.o)
P_OBJS += $(P_SRCS:.s=.o)
LIB = stdperiph.a
LIB_PATH = $(BASEDIR)/$(LIB)
LIBS= -lc -lgcc -lnosys -lm

all:mkdir lib proj
mkdir:
#	mkdir $(OUTPATH)
%.o : %.c    
	$(CC) $(CFLAGS) -c -o $@ $^ $(INCLUDE_PATHS)  
lib:$(OBJS)
	$(AR) -r $(LIB) $(OBJS)
proj:$(OUTPATH)/$(PROJ_NAME).elf
$(OUTPATH)/$(PROJ_NAME).elf:$(P_OBJS)
	$(CC) $(LDFLAGS) -T$(LINKER_SCRIPT) $(LIBPATHS) -o $@ $^ $(LIB_PATH) $(LIBS) $(LD_SYS_LIBS)

	
clean:
	rm $(OBJS)
	rm $(P_OBJS)
#	rm -r $(OUTPATH)
	rm -r $(LIB)