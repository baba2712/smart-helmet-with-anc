# ANC helmet firmware - STM32H743VIT6
#   make            build build/anc_helmet.elf/.bin/.hex
#   make flash      program over SWD (ST-Link, needs st-flash or STM32_Programmer_CLI)
#   make dfu        program over USB DFU (hold BOOT0 or send 'dfu' on the CLI)
#   make test       host tests: firmware DSP vs the Python simulation (needs python3 + numpy/scipy/numba)
#   make size       memory usage

PROJECT  = anc_helmet
BUILD    = build
TP       = third_party
CMSIS_DEV = $(TP)/cmsis_device_h7
CMSIS    = $(TP)/CMSIS_6/CMSIS/Core/Include
TUSB     = $(TP)/tinyusb/src

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

MCU     = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFS    = -DSTM32H743xx -DHSE_VALUE=25000000 -DCFG_TUSB_MCU=OPT_MCU_STM32H7
INC     = -Iinc -Idsp -Isrc -I$(CMSIS_DEV)/Include -I$(CMSIS) -I$(TUSB)
WARN    = -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter
CFLAGS  = $(MCU) $(DEFS) $(INC) $(WARN) -std=gnu11 -O2 -g3 -ffunction-sections -fdata-sections \
          -ffp-contract=fast -fno-math-errno -fno-tree-loop-distribute-patterns -MMD -MP
LDFLAGS = $(MCU) -T ld/stm32h743vi.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map -Wl,--print-memory-usage \
          --specs=nano.specs --specs=nosys.specs -u _printf_float -Wl,--no-warn-rwx-segments

SRC_APP = src/main.c src/syscalls.c src/hw.c src/audio.c src/app.c src/store.c src/power_ui.c src/comms.c \
          dsp/anc_core.c dsp/dsp_misc.c
SRC_SYS = $(CMSIS_DEV)/Source/Templates/system_stm32h7xx.c
SRC_USB = $(TUSB)/tusb.c $(TUSB)/common/tusb_fifo.c $(TUSB)/device/usbd.c \
          $(TUSB)/class/cdc/cdc_device.c $(TUSB)/portable/synopsys/dwc2/dcd_dwc2.c \
          $(TUSB)/portable/synopsys/dwc2/dwc2_common.c
ASM     = $(CMSIS_DEV)/Source/Templates/gcc/startup_stm32h743xx.s

OBJ = $(addprefix $(BUILD)/,$(SRC_APP:.c=.o) $(notdir $(SRC_SYS:.c=.o)) $(addprefix usb/,$(notdir $(SRC_USB:.c=.o))) \
      $(notdir $(ASM:.s=.o)))

all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin $(BUILD)/$(PROJECT).hex

$(BUILD)/src/%.o: src/%.c | dirs
	$(CC) $(CFLAGS) -Werror -c $< -o $@
$(BUILD)/dsp/%.o: dsp/%.c | dirs
	$(CC) $(CFLAGS) -Werror -c $< -o $@
$(BUILD)/system_stm32h7xx.o: $(SRC_SYS) | dirs
	$(CC) $(CFLAGS) -w -c $< -o $@
vpath %.c $(sort $(dir $(SRC_USB)))
$(BUILD)/usb/%.o: %.c | dirs
	$(CC) $(CFLAGS) -w -c $< -o $@
$(BUILD)/startup_stm32h743xx.o: $(ASM) | dirs
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) -lm -o $@
	$(SIZE) $@
$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@
$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

dirs:
	@mkdir -p $(BUILD)/src $(BUILD)/dsp $(BUILD)/usb

size: $(BUILD)/$(PROJECT).elf
	$(SIZE) -A $< | grep -E "itcm|data|bss|text|axisram|isr"

flash: $(BUILD)/$(PROJECT).bin
	st-flash --reset write $< 0x08000000 || STM32_Programmer_CLI -c port=SWD -w $< 0x08000000 -v -rst

dfu: $(BUILD)/$(PROJECT).bin
	dfu-util -a 0 -s 0x08000000:leave -D $<

test:
	python3 test/test_fil.py

clean:
	rm -rf $(BUILD) test/build

.PHONY: all dirs size flash dfu test clean

-include $(OBJ:.o=.d)
