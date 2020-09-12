## Copyright 2017 Inca Roads LLC.  All rights reserved.
## Use of this source code is governed by licenses granted by the
## copyright holder including that found in the LICENSE file.

#   TT Node Firmware
#
#	Structured for Mac development, using
#	the Nordic SDK and the GCC ARM support
#
#	Currently supports Nordic SDK version
#		SDK12.2
#
# Prerequisites:
# 1. Install gcc-arm-none-eabi from the ARM developer site
#    https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
# 2. brew install srecord (to get srec_cat)
# 3. brew install python
# 4. pip install nrfutil
#

APPNAME := scnano
PRODUCTION := true

MAJORVERSION := 1
MINORVERSION := 254

## Disable implicit rules
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:

# Install Nordic SDKs from https://developer.nordicsemi.com/nRF5_SDK/
# For example, for SDK 12.2, the path to the SDK and its doc are:
# https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v12.x.x/nRF5_SDK_12.2.0_f012efa.zip
# https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v12.x.x/nRF5_SDK_12.2.0_offline_doc.zip
# Then, download them into a folder, unzip it, and change this SDKROOT to be the folder
# where you unzipped it.
SDKROOT := /home/rob/Documents/PlatformIO/Projects/TTNode/SDK/
# Install the protocol buffers SDK, Mac release, from https://koti.kapsi.fi/jpa/nanopb/
# Then, set this path to where the nanopb folder exists (with its README.md in it, etc.)
PBSDK := /home/rob/Documents/Applications/nanopb-0.4.1-linux-x86

ifeq ($(APPNAME),breadboard)
BOARD := scv0
DFU := NODFU
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
# This is the breadboard using FONA GPS and 1 tube
#PERIPHERAL_DEFS := -DLABEL=ray-breadboard -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DUSGPS=USAB -DGEIGERX -DG0=LND7318U -DTWIX -DTWIBME280X -DTWIBME0 -DTWIINA219 -DMOTIONX -DTWILIS3DH -DAIRX -DPMSX=IOUART -DPMS5003 -DSPIX -DSPIOPC -DLORA -DCELLX -DFONA -DFONAGPS -DTESTDEVICE
# This is breadboard using UGPS and INA219 and 1 tube
PERIPHERAL_DEFS := -DSSD -DLABEL=ray-breadboard -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DUSGPS=USAB -DGEIGERX -DG0=LND7318U -DTWIX  -DTWIBME280X -DTWIBME0 -DTWIINA219 -DMOTIONX -DTWILIS3DH -DAIRX -DPMSX=IOUART -DPMS5003 -DSPIX -DSPIOPC -DLORA -DCELLX -DFONA -DUGPS -DTESTDEVICE
#DEBUG_DEFS := -DBURN -DCELL15DEBUG
#DEBUG_DEFS := -DTWIBME1
#DEBUG_DEFS := -DSTORAGE_WAN=WAN_FONA -DCOMMS_FORCE_NONBUFFERED -DROCKSGPS -DBTKEEPALIVE -DCOMMDEBUG
#DEBUG_DEFS := -DSTORAGE_WAN=WAN_FONA -DBATTERY_NORMAL -DAIR_COUNTS
#DEBUG_DEFS := -DSTORAGE_WAN=WAN_FONA -DHOURLYSTATS -DAIR_COUNTS
#DEBUG_DEFS := -DSTORAGE_WAN=WAN_LORAWAN
#DEBUG_DEFS := -DBTKEEPALIVE -DAIR_COUNTS
#DEBUG_DEFS := -DBTKEEPALIVE -DAIR_COUNTS -DSTORAGE_WAN=WAN_FONA
##DEBUG## Uncomment this when you want to debug with a serial cable
#DEBUG_DEFS := -DDEBUG_USES_UART -DNRF_LOG_USES_RTT=1 -DENABLE_DEBUG_LOG_SUPPORT
##DEBUG## Uncomment this to do power draw testing of the motion detector
#PERIPHERAL_DEFS := -DTWIX -DTWILIS3DH
#DEBUG_DEFS := -DMOTIONDEBUG
##DEBUG## Uncomment this to do Absolute minimum power draw testing
#PERIPHERAL_DEFS := -DPOWERDEBUG
#DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE -DSSDDEBUG
#DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE -DBURN -DSSDKEEPALIVE -DBURN
#DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE -DSSDKEEPALIVE -DBURN -DBURNLORA -DBURNCOMMS
#DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE -DBURN -DSCHEDDEBUG -DBURNFONA -DAIR_ZERO_TEST
ifeq ($(PRODUCTION),true)
DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE
DFU := DFU
DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE
endif
endif

ifeq ($(APPNAME),solarcast)
BOARD := scv1
DFU := NODFU
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
PERIPHERAL_DEFS := -DSSD -DSSD_UPSIDE_DOWN -DOCSENSE -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DUSGPS=USAB -DGEIGERX -DG0=LND7318U -DG1=LND7318C -DTWIX  -DTWIBME280X -DTWIBME0 -DTWIBME1 -DTWIBME0AIR -DTWIMAX17201 -DMOTIONX -DTWILIS3DH -DAIRX -DPMSX=IOUART -DPMS5003 -DSPIX -DSPIOPC -DLORA -DCELLX -DFONA -DUGPS
DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE -DBURN -DSCHEDDEBUG
## For power testing
#PERIPHERAL_DEFS := -DPOWERDEBUG
#PERIPHERAL_DEFS := -DPOWERDEBUG -DPOWERDEBUG_OUTPUTS_ON
## This must be last in this section
ifeq ($(PRODUCTION),true)
# DFU := DFU 
DEBUG_DEFS := -DAIR_COUNTS -DBTKEEPALIVE
endif
endif

ifeq ($(APPNAME),musti)
PRODUCTION := false
BOARD := scv1
DFU := NODFU
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
PERIPHERAL_DEFS := -DSPIX -DSPIOPC 
DEBUG_DEFS := -DBTKEEPALIVE -DFAST -DSCHEDDEBUG -DSENSORDEBUG -DSENSORMAXDEBUG
DEBUG_DEFS := -DBTKEEPALIVE -DFAST -DSCHEDDEBUG -DSENSORDEBUG -DSENSORMAXDEBUG -DBTDEBUG_BYPASS_BUFFERING
endif

ifeq ($(APPNAME),scnano)
BOARD := scv2
DFU := NODFU
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
PERIPHERAL_DEFS := -DSSD -DHWFC=false -DGEIGERX -DG0=LND7318U -DG1=LND7128EC -DTWIX -DTWIINA219 -DMOTIONX -DTWILIS3DH -DCELLX -DFONA -DUGPS -DSTORAGE_WAN=WAN_FONA
ifeq ($(BOARD),scv0)
PERIPHERAL_DEFS += -DUSX -DUSFONA=USAb -DUSGPS=USAB -DLABEL=ray-bbnano
endif
# ifeq ($(PRODUCTION),true)
# DFU := DFU
# endif
#DEBUG_DEFS := -DBTKEEPALIVE
endif

ifeq ($(APPNAME),prototype)
BOARD := scv0
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
DFU := NODFU
DFU := DFU
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
PERIPHERAL_DEFS := -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DGEIGERX -DG0=LND7318U -DG1=LND7128EC -DTWIX  -DTWIBME280X -DTWIBME0 -DTWIINA219 -DMOTIONX -DTWILIS3DH -DAIRX -DPMSX=IOUART -DPMS5003 -DSPIX -DSPIOPC -DLORA -DCELLX -DFONA -DFONAGPS
DEBUG_DEFS := -DLABEL=ray-proto -DAIR_COUNTS
endif

ifeq ($(APPNAME),homecast)
BOARD := scv0
NSDKVER := NSDKV122
SOFTDEVICE := S132
BONDING := NOBONDING
MCU := NRF52
DFU := NODFU
MCU_DEFS := -DCONFIG_NFCT_PINS_AS_GPIOS
NRF_DEFS := -DNRF52832 -DNRF_SD_BLE_API_VERSION=3
CPU_DEFS := -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mabi=aapcs
PERIPHERAL_DEFS := -DLABEL=ray-homecast -DUSX -DUSLORA=USab -DUSFONA=USAb -DUSPMS=USaB -DPMSX=IOUART -DPMS5003 -DLORA -DHWFC=false -DAIR_COUNTS -DTESTDEVICE
#DEBUG_DEFS := -DBTKEEPALIVE -DSTORAGE_WAN=WAN_LORAWAN
DEBUG_DEFS := -DBTKEEPALIVE -DSTORAGE_WAN=WAN_LORA
endif

## Simplecast
ifeq ($(APPNAME),simplecast)
BOARD := blenano
NSDKVER := NSDKV11
SOFTDEVICE := S130
BONDING := NOBONDING
MCU := NRF51
DFU := NODFU
MCU_DEFS :=
CPU_DEFS := -mcpu=cortex-m0 -mfloat-abi=soft -mthumb -mabi=aapcs
NRF_DEFS := -DNRF51822 -DNRF_SD_BLE_API_VERSION=2
PERIPHERAL_DEFS := -DLABEL=ray-simplecast -DGEIGERX -DG0=LND7318U -DG1=LND7128EC -DBGEIGIE -DTWIX -DTWIMAX17043 -DTWIHIH6130 -DTWIUBLOXM8 -DLORA
# as of Feb 2017 the GPS has failed
DEBUG_DEFS := -DROCKSGPS -DBTKEEPALIVE
##DEBUG## Uncomment this when you want to debug with a serial cable
#DEBUG_DEFS := -DDEBUG_USES_UART -DNRF_LOG_USES_RTT=1 -DENABLE_DEBUG_LOG_SUPPORT
endif

## echo Makefile debugging
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

## In SDK12 they broke the file extensions
ifeq ("$(NSDKVER)","NSDKV11")
ASMEXT := s
else
ASMEXT := S
endif

###### GCC
# Install gcc-arm using this, and find the path where it was installed
#GNU_INSTALL_ROOT :=/usr/
#GNU_PREFIX := arm-none-eabi
#####

##### Nano Protocol Buffers SDK
#####

##### Hardware definitions
HARDWARE_DIRECTORY := ./board
#####

### Nordic SDK
ifeq ("$(NSDKVER)","NSDKV11")
NSDK := $(SDKROOT)/nRF5_SDK_11.0.0_89a8197
SOFTDEVICE_PATH := $(NSDK)/components/softdevice/$(SOFTDEVICE)/hex/$(SOFTDEVICE)_$(MCU)_2.0.0_softdevice.hex
TEMPLATE_PATH := $(NSDK)/components/toolchain/gcc
LINKER_SCRIPT := $(HARDWARE_DIRECTORY)/memory/$(MCU)_$(SOFTDEVICE)_sdkv11_$(DFU).ld
endif
ifeq ("$(NSDKVER)","NSDKV121")
NSDK := $(SDKROOT)/nRF5_SDK_12.1.0_0d23e2a
SOFTDEVICE_PATH := $(NSDK)/components/softdevice/$(SOFTDEVICE)/hex/$(SOFTDEVICE)_$(MCU)_3.0.0_softdevice.hex
TEMPLATE_PATH := $(NSDK)/components/toolchain/gcc
LINKER_SCRIPT := $(HARDWARE_DIRECTORY)/memory/$(MCU)_$(SOFTDEVICE)_sdkv12_$(DFU).ld
endif
ifeq ("$(NSDKVER)","NSDKV122")
NSDK := $(SDKROOT)/nRF5_SDK_12.2.0_f012efa
SOFTDEVICE_PATH := $(NSDK)/components/softdevice/$(SOFTDEVICE)/hex/$(SOFTDEVICE)_$(MCU)_3.0.0_softdevice.hex
TEMPLATE_PATH := $(NSDK)/components/toolchain/gcc
LINKER_SCRIPT := $(HARDWARE_DIRECTORY)/memory/$(MCU)_$(SOFTDEVICE)_sdkv12_$(DFU).ld
endif
###

# Bootloader directory
ifeq ($(DFU),NODFU)
BOOTLOADER_PATH := 
else
BOOTLOADER_PATH := ./ttboot/bin/ttboot-$(BOARD).hex
endif

# This is the Mac volume that will be used when doing a 'make flash'
ifeq ($(BOARD),scv2)
FLASHDEVICE = /Volumes/DAPLINK
else
FLASHDEVICE = /Volumes/MBED
endif

# Various folders used within this makefile - end of configuration
SOURCE_DIRECTORY := src
OBJECT_DIRECTORY := bin
BUILD_DIRECTORY := build
DFU_DIRECTORY := dfu
OUTPUT_FILENAME := $(APPNAME)
LISTING_DIRECTORY := $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY := $(OBJECT_DIRECTORY)
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) $(BUILDPATH) )
BUILDTIME := $(shell date -ju "+%Y/%m/%d %H:%M:%S")
BUILDVERSION := $(shell expr $(shell cat $(BUILD_DIRECTORY)/build_version) + 1 )
APPVERSION := $(MAJORVERSION).$(MINORVERSION).$(BUILDVERSION)
BUILDFILE := $(APPNAME)-$(MAJORVERSION)-$(MINORVERSION)
BUILDPATH := $(BUILD_DIRECTORY)/$(BUILDFILE)
DEVPATH := $(OUTPUT_BINARY_DIRECTORY)/hex/$(BUILDFILE)-$(BUILDVERSION)

# We COULD use the true app version for DFU, however the Nordic SDK code prevents us from
# going backward in version numbers when updating via DFU.  As such, we explicitly have decided
# to set the DFU version number to 1.1 always - to enable upgrade or downgrade as we see fit.
#DFUAPPVERSION := $(shell expr $(MAJORVERSION) \* 65536 + $(MINORVERSION) )
DFUAPPVERSION := $(shell expr 1 \* 65536 + 1 )


# Toolchain commands
MK				:= mkdir
RM 				:= rm -rf
CP				:= cp
CC              := '/usr/bin/arm-none-eabi-gcc'
AS              := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/usr/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/usr//bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_FILES = $(shell find $(SOURCE_DIRECTORY) -name '*.c')
C_FILES += $(shell find $(HARDWARE_DIRECTORY) -name '*.c')
C_SOURCE_FILES = \
$(C_FILES) \
$(PBSDK)/pb_common.c \
$(PBSDK)/pb_decode.c \
$(PBSDK)/pb_encode.c \
$(NSDK)/components/libraries/fifo/app_fifo.c \
$(NSDK)/components/libraries/timer/app_timer.c \
$(NSDK)/components/libraries/timer/app_timer_appsh.c \
$(NSDK)/components/libraries/twi/app_twi.c \
$(NSDK)/components/libraries/uart/app_uart_fifo.c \
$(NSDK)/components/libraries/gpiote/app_gpiote.c \
$(NSDK)/components/libraries/util/sdk_mapped_flags.c \
$(NSDK)/components/libraries/fstorage/fstorage.c \
$(NSDK)/components/libraries/util/app_util_platform.c \
$(NSDK)/components/libraries/scheduler/app_scheduler.c \
$(NSDK)/components/libraries/crc32/crc32.c \
$(NSDK)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
$(NSDK)/components/drivers_nrf/spi_master/nrf_drv_spi.c \
$(NSDK)/components/drivers_nrf/clock/nrf_drv_clock.c \
$(NSDK)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(NSDK)/components/drivers_nrf/ppi/nrf_drv_ppi.c \
$(NSDK)/components/drivers_nrf/common/nrf_drv_common.c \
$(NSDK)/components/drivers_nrf/uart/nrf_drv_uart.c \
$(NSDK)/components/ble/ble_advertising/ble_advertising.c \
$(NSDK)/components/ble/ble_db_discovery/ble_db_discovery.c \
$(NSDK)/components/ble/common/ble_advdata.c \
$(NSDK)/components/ble/common/ble_conn_params.c \
$(NSDK)/components/ble/common/ble_conn_state.c \
$(NSDK)/components/ble/common/ble_srv_common.c \
$(NSDK)/components/toolchain/system_$(shell echo $(MCU) | tr A-Z a-z).c \
$(NSDK)/components/softdevice/common/softdevice_handler/softdevice_handler.c
ifeq ("$(NSDKVER)","NSDKV11")
C_SOURCE_FILES += \
$(NSDK)/components/ble/device_manager/device_manager_peripheral.c \
$(NSDK)/components/drivers_nrf/delay/nrf_delay.c \
$(NSDK)/components/drivers_nrf/pstorage/pstorage.c
else
C_SOURCE_FILES += \
$(NSDK)/components/libraries/log/src/nrf_log_backend_serial.c \
$(NSDK)/components/libraries/log/src/nrf_log_frontend.c
endif
ifeq ("$(DFU)","DFU")
ifeq ("$(NSDKVER)","NSDKV11")
C_SOURCE_FILES += \
$(NSDK)/components/ble/ble_services/ble_dfu/ble_dfu.c \
$(NSDK)/components/libraries/bootloader_dfu/bootloader_util.c \
$(NSDK)/components/libraries/bootloader_dfu/dfu_app_handler.c
else
C_SOURCE_FILES += \
$(NSDK)/components/ble/ble_services/ble_dfu/ble_dfu.c \
$(NSDK)/components/libraries/bootloader/dfu/nrf_dfu_settings.c
endif
endif
ifeq ("$(BONDING)","BONDING")
C_SOURCE_FILES += \
$(NSDK)/components/ble/peer_manager/peer_manager.c \
$(NSDK)/components/ble/peer_manager/peer_database.c \
$(NSDK)/components/ble/peer_manager/peer_id.c \
$(NSDK)/components/ble/peer_manager/peer_data.c \
$(NSDK)/components/ble/peer_manager/peer_data_storage.c \
$(NSDK)/components/ble/peer_manager/gatt_cache_manager.c \
$(NSDK)/components/ble/peer_manager/gatts_cache_manager.c \
$(NSDK)/components/ble/peer_manager/security_manager.c \
$(NSDK)/components/ble/peer_manager/security_dispatcher.c \
$(NSDK)/components/ble/peer_manager/id_manager.c \
$(NSDK)/components/ble/peer_manager/pm_buffer.c \
$(NSDK)/components/ble/peer_manager/pm_mutex.c \
$(NSDK)/components/libraries/fds/fds.c
endif

#assembly files common to all targets
ASM_SOURCE_FILES  = $(NSDK)/components/toolchain/gcc/gcc_startup_$(shell echo $(MCU) | tr A-Z a-z).$(ASMEXT)

#include all folders containing source files, for convenience, before adding more
H_FILES = $(shell find $(SOURCE_DIRECTORY) -name '*.h')
H_FILES += $(shell find $(HARDWARE_DIRECTORY) -name '*.h')
H_FILES += $(C_SOURCE_FILES)
H_PATHS = $(call remduplicates, $(dir $(H_FILES) ) )
INC_PATHS = $(H_PATHS:%=-I%)
INC_PATHS += -I$(NSDK)/components/drivers_nrf/config
INC_PATHS += -I$(NSDK)/components/drivers_nrf/gpiote
INC_PATHS += -I$(NSDK)/components/libraries/scheduler
INC_PATHS += -I$(NSDK)/components/drivers_nrf/hal
INC_PATHS += -I$(NSDK)/components/drivers_nrf/rng
INC_PATHS += -I$(NSDK)/components/drivers_nrf/delay
INC_PATHS += -I$(NSDK)/components/libraries/button
INC_PATHS += -I$(NSDK)/components/libraries/util
INC_PATHS += -I$(NSDK)/components/libraries/log
INC_PATHS += -I$(NSDK)/components/libraries/log/src
INC_PATHS += -I$(NSDK)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(NSDK)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(NSDK)/components/device
INC_PATHS += -I$(NSDK)/components/softdevice/$(SOFTDEVICE)/headers
ifneq ("$(NSDKVER)","NSDKV11")
INC_PATHS += -I$(NSDK)/components/softdevice/$(SOFTDEVICE)/headers/nrf52
INC_PATHS += -I$(NSDK)/components/libraries/bootloader/dfu
endif
INC_PATHS += -I$(NSDK)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(NSDK)/components/libraries/fds/config
INC_PATHS += -I$(NSDK)/components/libraries/fstorage/config
INC_PATHS += -I$(NSDK)/components/libraries/experimental_section_vars
INC_PATHS += -I$(NSDK)/components/toolchain/gcc
INC_PATHS += -I$(NSDK)/components/libraries/trace

## Compiler & Linker Flags common to both C and Assembler

# Required for any debugging at all
CAFLAGS += -DDEBUG
# System configuration
CAFLAGS += -D$(BONDING)
CAFLAGS += -D$(NSDKVER)
CAFLAGS += -DSWI_DISABLE0
CAFLAGS += -DSOFTDEVICE_PRESENT
CAFLAGS += -D$(SOFTDEVICE)
CAFLAGS += -DBLE_STACK_SUPPORT_REQD
CAFLAGS += $(NRF_DEFS)
# MCU-specific definitions
CAFLAGS += -D$(MCU)
CAFLAGS += -DBOARD_CUSTOM
CAFLAGS += $(MCU_DEFS)
CAFLAGS += $(DEBUG_DEFS)
# DFU
CAFLAGS += -D$(DFU)
CAFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CAFLAGS += -DFIRMWARE=$(BUILDFILE)
ifneq ($(DFU),NODFU)
CAFLAGS += -DSTORAGE_DFU_FIRMWARE=$(BUILDFILE)
endif
# App feature configuration
CAFLAGS += -D$(BOARD)
CAFLAGS += $(STORAGE_DEFS)
CAFLAGS += $(PERIPHERAL_DEFS)
CAFLAGS += -DSTORAGE_LABEL=$(APPNAME)
CAFLAGS += -DAPPVERSION=$(APPVERSION) -DAPPMAJOR=$(MAJORVERSION) -DAPPMINOR=$(MINORVERSION) -DAPPBUILD=$(BUILDVERSION)

## C compiler flags
CFLAGS += $(CAFLAGS)
CFLAGS += $(CPU_DEFS) --std=gnu99
CFLAGS += -Wall -Werror -O3
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

## Assembler flags
ASMFLAGS += $(CAFLAGS)
ASMFLAGS += -x assembler-with-cpp

## Linker flags
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += $(CPU_DEFS) -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
# allow floats in printf
LDFLAGS += -u _printf_float

## Source & Object Files
C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )
ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.$(ASMEXT)=.o) )
vpath %.c $(C_PATHS)
vpath %.$(ASMEXT) $(ASM_PATHS)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

## The first rule is the default dependency
default: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo
	@echo "  \"make flash\" to transfer $(APPNAME) to $(FLASHDEVICE)"

## Create build directories
$(BUILD_DIRECTORIES):
	@echo Creating $@
	$(MK) $@

## Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

## Assemble files
$(OBJECT_DIRECTORY)/%.o: %.$(ASMEXT)
	@echo Assembling: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

## Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(OBJECTS) $(LINKER_SCRIPT) $(BOOTLOADER_PATH)
	@echo Linking: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $(OUTPUT_BINARY_DIRECTORY)/config.o $(SOURCE_DIRECTORY)/config.c
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Hexifying: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
ifeq ($(DFU),NODFU)
	@if [ -f  "$(BUILDPATH).zip" ]; then rm $(BUILDPATH).zip; fi
	@echo Packaging APP+SD for manual install: $(BUILDPATH).hex
	@srec_cat $(SOFTDEVICE_PATH) -intel $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME).hex -intel -o $(BUILDPATH).hex -intel --line-length=44
else
## Note that I got the list of FWID's here: https://devzone.nordicsemi.com/question/3629/how-do-i-access-softdevice-version-string/?answer=3693#post-id-3693
	@echo Generating bootloader settings:
	@nrfutil settings generate --family NRF52 --application-version $(DFUAPPVERSION) --bootloader-version 1 --bl-settings-version 1 --application $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)-settings.hex
	@echo Package bootloader settings into bootloader image: $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME)-bootloader.hex
	@srec_cat -o $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME)-bootloader.hex -intel $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)-settings.hex -intel $(BOOTLOADER_PATH) -intel --line-length=44
	@echo Packaging APP for over-the-air update
	@cp $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex $(OUTPUT_BINARY_DIRECTORY)/dfu.hex
	@nrfutil pkg generate --key-file $(DFU_DIRECTORY)/$(APPNAME).pem --application-version  $(DFUAPPVERSION) --hw-version 52 --sd-req 0x81,0x88,0x8c --application $(OUTPUT_BINARY_DIRECTORY)/dfu.hex $(BUILDPATH).zip
	@unzip -o $(BUILDPATH).zip -d $(BUILDPATH)
	@rm $(BUILDPATH)/manifest.json
	@echo Packaging APP+SD+BL for manual install: $(BUILDPATH).hex
	@srec_cat  $(SOFTDEVICE_PATH) -intel $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME).hex -intel $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME)-bootloader.hex -intel -o $(BUILDPATH).hex -intel --line-length=44
endif
## Done
	@echo Making dev copy as $(DEVPATH).hex
	@if [ ! -d "$(OUTPUT_BINARY_DIRECTORY)/hex" ]; then $(MK) $(OUTPUT_BINARY_DIRECTORY)/hex; fi
	@rm -f $(OUTPUT_BINARY_DIRECTORY)/hex/*
	@cp $(BUILDPATH).hex $(DEVPATH).hex
	@echo Logging build: $(APPNAME) $(APPVERSION)
	@echo $(BUILDVERSION) >$(BUILD_DIRECTORY)/build_version
	@echo $(BUILDTIME) UTC $(BOARD) $(APPNAME) $(APPVERSION) >>$(BUILD_DIRECTORY)/build_version.log

## Force clean build
clean:
	@echo Removing and re-creating $(BUILD_DIRECTORIES)
	$(RM) $(BUILD_DIRECTORIES)
	$(MK) $(BUILD_DIRECTORIES)
	@if [ ! -d "$(OUTPUT_BINARY_DIRECTORY)/hex" ]; then $(MK) $(OUTPUT_BINARY_DIRECTORY)/hex; fi

## Refresh ttproto from github
ttproto:
	$(RM) -rf ttproto
	git clone https://github.com/safecast/ttproto
	$(CP) ttproto/clang/* src/ttproto
	$(RM) -rf ttproto

## Push builds to ttserve for pulling by devices
push:
#	gcloud compute copy-files ./build/* rozzie@teletype-1:efs/safecast/build
	scp -i ~/dev/keys/safecastdev.pem -r ./build/* ubuntu@tt-ftp.safecast.org:~/efs/safecast/build

## This is ONLY needed if we want to play with FTPS; it's not needed in production
pushcert:
	gcloud compute copy-files ./cert/ftp/* rozzie@teletype-1:safecast/cert/ftp

## Flash
flash: $(OBJECT_DIRECTORY)/$(OUTPUT_FILENAME).hex
	@echo Flashing: $(BUILDPATH).hex to $(FLASHDEVICE)
	@until [ -d  "$(FLASHDEVICE)" ]; do echo "Waiting for device..."; sleep 2s; done
	@sleep 2
	$(CP) -X $(BUILDPATH).hex $(FLASHDEVICE)
	@until [ ! -d  "$(FLASHDEVICE)" ]; do echo "Flashing - waiting for dismount..."; sleep 1s; done
	@until [ -d  "$(FLASHDEVICE)" ]; do echo "Flashing - waiting for mount..."; sleep 2s; done
	@if [ -f  "$(FLASHDEVICE)/fail.txt" ]; then cat $(FLASHDEVICE)/fail.txt && echo "" && echo "** FAILURE! **"; else diskutil unmount $(FLASHDEVICE); fi
	@echo Done.

## DFU
#
# For info about the how/why of nrfutil, see:
#    https://github.com/NordicSemiconductor/pc-nrfutil/
#    http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v13.0.0%2Fexamples_bootloader.html&cp=4_0_0_4_3
 $@

dfuhelp:
	nrfutil --help
	nrfutil keys --help
	nrfutil pkg generate --help

dfukey:
## SDK12
	nrfutil keys generate $(DFU_DIRECTORY)/$(APPNAME).pem

## End
