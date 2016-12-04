#############################################################
# Required variables for each makefile
# Discard this section from all parent makefiles
# Expected variables (with automatic defaults):
#   CSRCS (all "C" files in the dir)
#   SUBDIRS (all subdirs with a Makefile)
#   GEN_LIBS - list of libs to be generated ()
#   GEN_IMAGES - list of object file images to be generated ()
#   GEN_BINS - list of binaries to be generated ()
#   COMPONENTS_xxx - a list of libs/objs in the form
#     subdir/lib to be extracted and rolled up into
#     a generated lib/image xxx.a ()
#
TARGET = eagle
#FLAVOR = release
FLAVOR = debug

#ESP8266 config
BOOT=new
APP=1
SPI_SPEED=80
SPI_MODE=QIO
SPI_SIZE_MAP=6

#Tag for OTA images. 0-27 characters. Change to eg your projects title.
OTA_TAGNAME ?= espway

ESPTOOL ?= esptool.py
ESPPORT ?= /dev/ttyUSB0
ESPDELAY	?= 3
ESPBAUD		?= 460800

define maplookup
$(patsubst $(strip $(1)):%,%,$(filter $(strip $(1)):%,$(2)))
endef

ESPTOOL_SIZE=$(call maplookup,$(SPI_SIZE_MAP),0:4m 2:8m 3:16m 4:32m 5:16m 6:32m)
ESPTOOL_MODE=$(call maplookup,$(SPI_MODE),QIO:qio QOUT:qout DIO:dio DOUT:dout)
ESPTOOL_FLASHDEF=--flash_freq $(SPI_SPEED)m --flash_mode $(ESPTOOL_MODE) --flash_size $(ESPTOOL_SIZE)
ESPTOOL_OPTS=--port $(ESPPORT) --baud $(ESPBAUD)
OTA_FLASH_SIZE_K=$(call maplookup,$(SPI_SIZE_MAP),0:4096 2:8192 3:16384 4:32768 5:16384 6:32768)
ESP_TGT_BLANK=0x3FE000 "$(SDK_PATH)/bin/blank.bin" 0x3FC000 $(SDK_PATH)/bin/esp_init_data_default.bin
ESP_TGT_FLASH=0x00000 "$(SDK_PATH)/bin/boot_v1.6.bin" 0x1000 $(BIN_PATH)/upgrade/$(BIN_NAME).bin
ESPTOOL_OPTS=--port $(ESPPORT) --baud $(ESPBAUD)
EXTRA_CCFLAGS += -DOTA_FLASH_SIZE_K=$(OTA_FLASH_SIZE_K) -DOTA_TAGNAME="\"$(OTA_TAGNAME)\""

ifndef PDIR # {
GEN_IMAGES= eagle.app.v6.out
GEN_BINS= eagle.app.v6.bin
SPECIAL_MKTARGETS=$(APP_MKTARGETS)
SUBDIRS=    \
	user

endif # } PDIR

LDDIR = $(SDK_PATH)/ld

CCFLAGS += -Os

TARGET_LDFLAGS =		\
	-nostdlib		\
	-Wl,-EL \
	--longcalls \
	--text-section-literals

ifeq ($(FLAVOR),debug)
    TARGET_LDFLAGS += -ggdb -Og
endif

ifeq ($(FLAVOR),release)
    TARGET_LDFLAGS += -ggdb -O2
endif

dummy: all

libesphttpd/libesphttpd.a: libesphttpd/Makefile
	make -C libesphttpd libesphttpd.a FREERTOS=yes

libesphttpd/libwebpages-espfs.a: libesphttpd/Makefile
	make -C libesphttpd libwebpages-espfs.a FREERTOS=yes

flash: $(GEN_IMAGES) $(TARGET_OUT)
	$(ESPTOOL) $(ESPTOOL_OPTS) write_flash $(ESPTOOL_FLASHDEF) $(ESP_TGT_FLASH)

blankflash:
	$(ESPTOOL) $(ESPTOOL_OPTS) write_flash $(ESPTOOL_FLASHDEF) $(ESP_TGT_BLANK)

COMPONENTS_eagle.app.v6 = \
	user/libuser.a

DEPENDS_eagle.app.v6 = \
				$(LD_FILE) \
				libesphttpd/libesphttpd.a \
				libesphttpd/libwebpages-espfs.a


SDKLIBS=minic gcc hal phy pp net80211 wpa crypto main freertos lwip
LINKFLAGS_eagle.app.v6 = \
	-L$(SDK_PATH)/lib        \
	-Wl,--gc-sections   \
	-nostdlib	\
    -T$(LD_FILE)   \
	-Wl,--no-check-sections	\
    -u call_user_start	\
	-Wl,-static						\
	-Wl,--start-group					\
	$(addprefix -l,$(SDKLIBS)) \
	-L./libesphttpd \
	-lesphttpd \
	-lwebpages-espfs \
	$(DEP_LIBS_eagle.app.v6)					\
	-Wl,--end-group \
	-Wl,-Map=mapfile.txt



#############################################################
# Configuration i.e. compile options etc.
# Target specific stuff (defines etc.) goes in here!
# Generally values applying to a tree are captured in the
#   makefile at its root level - these are then overridden
#   for a subtree within the makefile rooted therein
#

#UNIVERSAL_TARGET_DEFINES =		\

# Other potential configuration flags include:
#	-DTXRX_TXBUF_DEBUG
#	-DTXRX_RXBUF_DEBUG
#	-DWLAN_CONFIG_CCX
CONFIGURATION_DEFINES =	-DICACHE_FLASH -DFREERTOS

DEFINES +=				\
	$(UNIVERSAL_TARGET_DEFINES)	\
	$(CONFIGURATION_DEFINES)

DDEFINES +=				\
	$(UNIVERSAL_TARGET_DEFINES)	\
	$(CONFIGURATION_DEFINES)


#############################################################
# Recursion Magic - Don't touch this!!
#
# Each subtree potentially has an include directory
#   corresponding to the common APIs applicable to modules
#   rooted at that subtree. Accordingly, the INCLUDE PATH
#   of a module can only contain the include directories up
#   its parent path, and not its siblings
#
# Required for each makefile to inherit from the parent
#

INCLUDES := $(INCLUDES) -I $(PDIR)include -I $(PDIR)libesphttpd/include -I $(PDIR)libesphttpd/espfs
sinclude $(SDK_PATH)/Makefile

.PHONY: FORCE
FORCE:

