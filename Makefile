PROGRAM = espway
SRC_DIR = ./src
PROGRAM_SRC_DIR = $(SRC_DIR) $(SRC_DIR)/lib
PROGRAM_INC_DIR = $(SRC_DIR)
BUILD_DIR = ./build/
FIRMWARE_DIR = ./firmware/

ESPBAUD ?= 460800
FLASH_SPEED ?= 80
FLASH_SIZE ?= 16
PRINTF_SCANF_FLOAT_SUPPORT ?= 0
SPLIT_SECTIONS ?= 0
WARNINGS_AS_ERRORS ?= 0
ENABLE_CXX ?= 0
FINAL_IMAGE_PREFIX = $(FIRMWARE_DIR)$(PROGRAM)-
FINAL_IMAGE = $(FINAL_IMAGE_PREFIX)0x00000.bin

N_PROCESSES = 5

EXTRA_COMPONENTS = extras/dhcpserver extras/httpd extras/mbedtls extras/i2c extras/i2s_dma extras/ws2812_i2s

EXTRA_C_CXX_FLAGS = -DLWIP_HTTPD_CGI=1 -Isrc -DESP_OPEN_RTOS
EXTRA_CXXFLAGS = -std=gnu++11

# FLAVOR = debug
# EXTRA_C_CXX_FLAGS += -DLWIP_DEBUG=1 -DHTTPD_DEBUG=LWIP_DBG_ON

all: fsdata $(FINAL_IMAGE)

fsdata: src/fsdata.c

src/fsdata.c: frontend/src/*
	cd frontend; npm run build
	perl scripts/makefsdata

clean: clean-fsdata

clean-fsdata:
	$(Q) rm -f src/fsdata.c

include esp-open-rtos/common.mk

parallel:
	$(MAKE) clean
	$(MAKE) fsdata
	$(MAKE) -j$(N_PROCESSES) $(PROGRAM_OUT)

PROGRAM_BIN = $(BUILD_DIR)$(PROGRAM).bin

$(PROGRAM_BIN): $(PROGRAM_OUT)
	$(OBJCOPY) -O binary $(PROGRAM_OUT) $(PROGRAM_BIN)

$(FINAL_IMAGE): $(PROGRAM_BIN) $(RBOOT_BIN) $(RBOOT_CONF) $(FIRMWARE_DIR)
	$(ESPTOOL) make_image \
		-f $(RBOOT_BIN) -a 0x0 -f $(RBOOT_CONF) -a 0x1000 -f $(PROGRAM_BIN) -a 0x2000 \
		$(FINAL_IMAGE_PREFIX)

flash-only:
	$(ESPTOOL) -p $(ESPPORT) --baud $(ESPBAUD) write_flash $(ESPTOOL_ARGS) \
		0x0 $(RBOOT_BIN) 0x1000 $(RBOOT_CONF) 0x2000 $(FW_FILE) $(SPIFFS_ESPTOOL_ARGS)
