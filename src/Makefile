PROGRAM_SRC_DIR=. ./lib

EXTRA_COMPONENTS=extras/dhcpserver extras/httpd extras/mbedtls extras/i2c

EXTRA_C_CXXFLAGS=-DLWIP_HTTPD_CGI=1 -DLWIP_DEBUG=1
EXTRA_CXXFLAGS=-std=gnu++11

include ../esp-open-rtos/common.mk
