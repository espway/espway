PROGRAM_SRC_DIR = . ./lib
PROGRAM_INC_DIR = .

ESPBAUD ?= 460800
FLASH_SPEED ?= 80
FLASH_SIZE ?= 16
PRINTF_SCANF_FLOAT_SUPPORT ?= 0
SPLIT_SECTIONS ?= 0
WARNINGS_AS_ERRORS ?= 0

EXTRA_COMPONENTS = extras/dhcpserver extras/httpd extras/mbedtls extras/i2c extras/i2s_dma extras/ws2812_i2s

# FLAVOR = debug
EXTRA_C_CXX_FLAGS = -DLWIP_HTTPD_CGI=1
EXTRA_CXXFLAGS = -std=gnu++11

include ../esp-open-rtos/common.mk

