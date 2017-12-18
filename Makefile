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

N_PROCESSES = 5

EXTRA_COMPONENTS = extras/dhcpserver extras/i2c extras/i2s_dma extras/ws2812_i2s

EXTRA_C_CXX_FLAGS = -DLWIP_HTTPD_CGI=1 -DHTTPD_FSDATA_FILE="\"fsdata_custom.c\"" \
	-Isrc -DESP_OPEN_RTOS
EXTRA_CXXFLAGS = -std=gnu++11

# FLAVOR = debug
# EXTRA_C_CXX_FLAGS += -DLWIP_DEBUG=1 -DHTTPD_DEBUG=LWIP_DBG_ON

include esp-open-rtos/common.mk

HTTPD_DIR = $(LWIP_DIR)apps/httpd/
FSDATA = $(SRC_DIR)/fsdata_custom.c

all: $(FSDATA)

$(FSDATA): $(HTTPD_DIR)makefsdata/makefsdata.c $(HTTPD_DIR)makefsdata/tinydir.h frontend/src/* 
	cd frontend; npm run build
	tcc -w -I$(lwip_ROOT)include -I$(LWIP_DIR)include -Iesp-open-rtos/core/include \
		-run $< frontend/output -f:$@

clean: clean-fsdata

clean-fsdata:
	$(Q) rm -f $(FSDATA)

parallel:
	$(MAKE) clean
	$(MAKE) $(FSDATA)
	$(MAKE) -j$(N_PROCESSES) all

flash-only:
	$(ESPTOOL) -p $(ESPPORT) --baud $(ESPBAUD) write_flash $(ESPTOOL_ARGS) \
		0x0 $(RBOOT_BIN) 0x1000 $(RBOOT_CONF) 0x2000 $(FW_FILE) $(SPIFFS_ESPTOOL_ARGS)
