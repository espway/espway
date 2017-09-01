PROGRAM = espway
SRC_DIR = ./src
PROGRAM_SRC_DIR = $(SRC_DIR) $(SRC_DIR)/lib
PROGRAM_INC_DIR = $(SRC_DIR)

ESPBAUD ?= 460800
FLASH_SPEED ?= 80
FLASH_SIZE ?= 16
PRINTF_SCANF_FLOAT_SUPPORT ?= 0
SPLIT_SECTIONS ?= 0
WARNINGS_AS_ERRORS ?= 0

N_PROCESSES = 5

EXTRA_COMPONENTS = extras/dhcpserver extras/httpd extras/mbedtls extras/i2c extras/i2s_dma extras/ws2812_i2s

EXTRA_C_CXX_FLAGS = -DLWIP_HTTPD_CGI=1 -DTCP_QUEUE_OOSEQ=0 -Itmp
EXTRA_CXXFLAGS = -std=gnu++11

# FLAVOR = debug
# EXTRA_C_CXX_FLAGS += -DLWIP_DEBUG=1 -DHTTPD_DEBUG=LWIP_DBG_ON

all: fsdata

tmp:
	mkdir tmp

clean: clean-fsdata

fsdata: tmp/fsdata.c

tmp/fsdata.c: tmp frontend/output/*
	cd frontend; npm run build
	perl scripts/makefsdata

clean-fsdata:
	$(Q) rm -rf tmp

parallel:
	$(MAKE) clean
	$(MAKE) fsdata
	$(MAKE) -j$(N_PROCESSES) all

include esp-open-rtos/common.mk

