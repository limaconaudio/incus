ifeq ($(DEBUG),yes)
	CFLAGS += -g3 -O0 -Wall -Wextra -Wno-unused-function
endif

ifeq ($(DEBUG),no)
	CFLAGS += -g -O3 
endif

ifeq ($(BENCH),yes)
	CFLAGS += -fno-inline  
endif

ifeq ($(SIFIVE_GCC_PACK),yes)
	RISCV_CLIB=$(RISCV_PATH)/$(RISCV_NAME)/lib/$(MARCH)/$(MABI)/
else
	RISCV_CLIB=$(RISCV_PATH)/$(RISCV_NAME)/lib/
endif

RISCV_OBJCOPY = $(RISCV_NAME)-objcopy
RISCV_OBJDUMP = $(RISCV_NAME)-objdump
RISCV_CC=$(RISCV_NAME)-gcc

CFLAGS +=  -MD -fstrict-volatile-bitfields 
LDFLAGS +=  -nostdlib -lgcc -mcmodel=medany -nostartfiles -ffreestanding -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,$(OBJDIR)/$(PROJ_NAME).map,--print-memory-usage

OBJDIR = build
OBJS := $(SRCS)
OBJS := $(OBJS:.c=.o)
OBJS := $(OBJS:.cpp=.o)
OBJS := $(OBJS:.S=.o)
OBJS := $(OBJS:..=miaou)
OBJS := $(addprefix $(OBJDIR)/,$(OBJS))

all: $(OBJDIR)/$(PROJ_NAME).elf $(OBJDIR)/$(PROJ_NAME).hex $(OBJDIR)/$(PROJ_NAME).asm $(OBJDIR)/$(PROJ_NAME).v

$(OBJDIR)/%.elf: $(OBJS) | $(OBJDIR)
	@echo "Linking $^"
	@$(RISCV_CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)

%.hex: %.elf
	@echo "Generating $@"
	@$(RISCV_OBJCOPY) -O ihex $^ $@

%.bin: %.elf
	@echo "Generating $@"
	@$(RISCV_OBJCOPY) -O binary $^ $@
	
%.v: %.elf
	@echo "Generating $@"
	@$(RISCV_OBJCOPY) -O verilog $^ $@
	
%.asm: %.elf
	@echo "Generating $@"
	@$(RISCV_OBJDUMP) -S -d $^ > $@

$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling $^"
	@$(RISCV_CC) -c $(CFLAGS)  $(INC) -o $@ $^
	
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $^"
	@$(RISCV_CC) -c $(CFLAGS)  $(INC) -o $@ $^	

$(OBJDIR)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "Compiling $^"
	@$(RISCV_CC) -c $(CFLAGS) -o $@ $^ -D__ASSEMBLY__=1

$(OBJDIR):
	@mkdir -p $@

clean:
	@rm -rf $(OBJDIR)

.SECONDARY: $(OBJS)
