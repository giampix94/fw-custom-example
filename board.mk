include $(BOARD_DIR)/firmware/firmware.mk

BOARDINC += $(BOARD_DIR)/generated/controllers/generated

# defines SHORT_BOARD_NAME
include $(BOARD_DIR)/meta-info.env

DDEFS += -DEFI_WIDEBAND_FIRMWARE_UPDATE=FALSE

# STM32F767ZIT6: 2048 KB di flash [ST STM32F767ZITx.xml]
include $(PROJECT_DIR)/hw_layer/ports/stm32/2mb_flash.mk

# Quarzo X1 = 8 MHz [netlist; BOM provaecu]. Nessun override di STM32_HSECLK:
# RusEFI F7 parte su HSI e rileva da solo la frequenza dell' HSE [osc_detector.cpp].

# Niente SD card sulla proto.
DDEFS += -DEFI_FILE_LOGGING=FALSE

# SD storage off (va spento insieme a FatFS)
DDEFS += -DEFI_STORAGE_SD=FALSE
