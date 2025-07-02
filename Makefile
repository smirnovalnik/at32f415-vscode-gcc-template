TARGET ?= target_name

-include .env

# Test
TEST ?= 0

VERSION_MAJOR = 0
VERSION_MINOR = 0
VERSION_PATCH = 1

HW_REVISION = 01

DEVICE = AT32F415C8T7

VERSION = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)
GIT_HASH = $(shell git rev-parse --short HEAD)

ifeq ($(TEST), 1)
	IMAGE_NAME = $(TARGET)-$(VERSION)-$(HW_REVISION)-$(GIT_HASH)-test
else
	IMAGE_NAME = $(TARGET)-$(VERSION)-$(HW_REVISION)-$(GIT_HASH)
endif

BUILD_DIR = build
OUT_DIR = out
DIST_DIR = dist
TMP_DIR = tmp

# Debug and optimization
DEBUG ?= 0
OPT = -O3

# Sources
PROJ_C_SOURCES = \
$(wildcard src/*.c) \
$(wildcard port/*.c) \

LIB_C_SOURCES = \
libs/at32f415/libraries/cmsis/cm4/device_support/system_at32f415.c \
$(wildcard libs/at32f415/libraries/drivers/src/*.c) \
libs/at32f415/middlewares/freertos/source/portable/GCC/ARM_CM3/port.c \
$(wildcard libs/at32f415/middlewares/freertos/source/*.c) \
libs/at32f415/middlewares/freertos/source/portable/memmang/heap_4.c \
libs/ulog/src/ulog.c \
$(wildcard libs/littlefs/*.c) \

C_SOURCES = $(PROJ_C_SOURCES) $(LIB_C_SOURCES)

# Asm
ASM_SOURCES = \
libs/at32f415/libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f415.s

PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

OPENOCD_PATH ?=
ATLINK_CONSOLE_PATH ?=

ATLINK_CONSOLE = $(ATLINK_CONSOLE_PATH)ATLink_Console

RM = rm -fR
MKDIR = mkdir -p
ifeq ($(OS),Windows_NT)
    SHA256SUM = CertUtil -hashfile
	ATLINK_CONSOLE = $(ATLINK_CONSOLE_PATH)ATLink_Console.exe
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        SHA256SUM = sha256sum
    endif
    ifeq ($(UNAME_S),Darwin)
        SHA256SUM = shasum -a 256
    endif
endif

# Flags
CPU = -mcpu=cortex-m4
FPU =
FLOAT-ABI =
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS =

C_DEFS =  \
-D$(DEVICE)\
-DTARGET=\"$(TARGET)\" \
-DVERSION=\"$(VERSION)\" \
-DVERSION_MAJOR=$(VERSION_MAJOR) \
-DVERSION_MINOR=$(VERSION_MINOR) \
-DVERSION_PATCH=$(VERSION_PATCH) \
-DHW_REVISION=\"$(HW_REVISION)\" \
-DGIT_HASH=\"$(GIT_HASH)\" \
-DTEST=$(TEST) \
-DDEBUG=$(DEBUG)
#-DNDEBUG \

# Includes
AS_INCLUDES =

C_INCLUDES =  \
-Isrc \
-Iport \
-Ilibs/at32f415/libraries/cmsis/cm4/device_support \
-Ilibs/at32f415/libraries/cmsis/cm4/core_support \
-Ilibs/at32f415/libraries/drivers/inc \
-Ilibs/at32f415/middlewares/freertos/source/include \
-Ilibs/at32f415/middlewares/freertos/source/portable/GCC/ARM_CM3 \
-Ilibs/ulog/src \
-Ilibs/littlefs \

# Result gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -std=c2x -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# LDFLAGS
LDSCRIPT = port/AT32F415x8_FLASH.ld

LIBDIR =
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(IMAGE_NAME).map,--cref -Wl,--gc-sections -Wl,--no-warn-rwx-segments

all: $(BUILD_DIR)/$(IMAGE_NAME).elf $(BUILD_DIR)/$(IMAGE_NAME).hex $(BUILD_DIR)/$(IMAGE_NAME).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) $(OUT_DIR)
	@echo Compiling $(notdir $< )
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo Assembling $(notdir $< )
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(IMAGE_NAME).elf: $(OBJECTS) Makefile
	@echo Linking $(notdir $@)
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@cp $@ $(OUT_DIR)
	@cp $@ $(OUT_DIR)/$(TARGET).elf
	@$(SZ) $(OUT_DIR)/$(IMAGE_NAME).elf
	@$(SHA256SUM) $(OUT_DIR)/$(IMAGE_NAME).elf | awk '{print $$1}' > $(OUT_DIR)/$(IMAGE_NAME).elf.sha256

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(HEX) $< $@
	@cp $@ $(OUT_DIR)
	@cp $@ $(OUT_DIR)/$(TARGET).hex
	@$(SHA256SUM) $(OUT_DIR)/$(IMAGE_NAME).hex | awk '{print $$1}' > $(OUT_DIR)/$(IMAGE_NAME).hex.sha256

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(BIN) $< $@
	@cp $@ $(OUT_DIR)
	@cp $@ $(OUT_DIR)/$(TARGET).bin
	@$(SHA256SUM) $(OUT_DIR)/$(IMAGE_NAME).bin | awk '{print $$1}' > $(OUT_DIR)/$(IMAGE_NAME).bin.sha256

$(BUILD_DIR):
	@mkdir -p $@

$(OUT_DIR):
	@mkdir -p $@

$(DIST_DIR):
	@mkdir -p $@

$(TMP_DIR):
	@mkdir -p $@

.PHONY: clean
clean:
	-rm -fR $(BUILD_DIR)
	-rm -fR $(OUT_DIR)
	-rm -fR $(DIST_DIR)
	-rm -fR $(TMP_DIR)

.PHONY: flash-openocd-atlink
flash-openocd-atlink:
	$(OPENOCD_PATH)/openocd -f interface/atlink_dap_v2.cfg -f target/at32f415xx.cfg -c "program $(OUT_DIR)/$(TARGET).elf verify reset exit"

.PHONY: reset-openocd-atlink
reset-openocd-atlink:
	$(OPENOCD_PATH)openocd -f interface/atlink_dap_v2.cfg -f target/at32f435xx.cfg -c "init; reset; exit"

.PHONY: flash-openocd-jlink
flash-openocd-jlink:
	$(OPENOCD_PATH)/openocd -f interface/jlink.cfg -f target/at32f415xx.cfg -c "program $(OUT_DIR)/$(TARGET).elf verify reset exit"

.PHONY: reset-openocd-jlink
reset-openocd-jlink:
	$(OPENOCD_PATH)/openocd -f interface/jlink.cfg -f target/at32f415xx.cfg -c "init; reset; exit"

.PHONY: flash-atlink-console
flash-atlink-console:
	$(ATLINK_CONSOLE) -device $(DEVICE) -connect -p --dfap --depp -d --fn $(OUT_DIR)/$(TARGET).hex --v

.PHONY: reset-atlink-console
reset-atlink-console:
	$(ATLINK_CONSOLE) -device $(DEVICE) -connect -r

.PHONY: test
test: clean
	gcc -Ilibs/unity/src -Isrc -o $(BUILD_DIR)/$(IMAGE_NAME)_test tests/unit_test_example.c libs/unity/src/unity.c
	$(BUILD_DIR)/$(IMAGE_NAME)_test

.PHONY: format-check
format-check:
	@clang-format --dry-run --Werror $(PROJ_C_SOURCES)

.PHONY: format
format:
	@clang-format -i $(PROJ_C_SOURCES)

.PHONY: docker-build
docker-build:
	@docker build --platform linux/amd64 -t at32-builder .

.PHONY: docker-run
docker-run:
	@docker run --rm -it -v $(shell pwd):/project at32-builder

.PHONY: docker-clean
docker-clean:
	@docker rmi at32-builder

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: build out dist tmp
