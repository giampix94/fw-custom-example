#!/bin/bash
# gen_config_custom.sh
#
# Override board-locale di gen_config_board.sh, registrato tramite CUSTOM_GEN_CONFIG in
# board.mk. Scopo: aggiungere -enumInputFile board_enums.h (i typedef C per i menu a
# tendina TC-CAN) SENZA modificare il submodule ext/rusefi, dove gen_config_board.sh ha
# la sua lista -enumInputFile cablata (righe 68-69) e non parametrizzabile da fuori.
#
# Hook verificato su rusefi_config.mk (submodule, pinnato
# a2d8784fb6f3599044cc0d7ec2c9fcb68912b2f7), regola .config-sentinel:
#   ifneq (,$(CUSTOM_GEN_CONFIG))
#       bash $(BOARD_DIR)/$(CUSTOM_GEN_CONFIG)
#   else
#       bash gen_config_board.sh $(BOARD_DIR) $(SHORT_BOARD_NAME)
#   endif
# Invocato da Make con CWD = firmware/ (stessa directory in cui gen_config_board.sh viene
# eseguito quando chiamato dalla stessa regola), quindi questo script NON fa cd: i path
# relativi (gen_config_common.sh, controllers/algo/...) sono relativi a firmware/ come
# nello script originale.

set -e

SCRIPT_NAME=$(basename "$0")

BOARD_DIR="$1"
SHORT_BOARD_NAME="$2"
INI="rusefi_${SHORT_BOARD_NAME}.ini"

source gen_config_common.sh
echo "$SCRIPT_NAME: Using COMMON_GEN_CONFIG [$COMMON_GEN_CONFIG]"

java \
 $COMMON_GEN_CONFIG_PREFIX \
 	-tool gen_config.sh \
 $COMMON_GEN_CONFIG \
	-enumInputFile controllers/algo/rusefi_hw_stm32_enums.h \
	-enumInputFile controllers/algo/rusefi_hw_adc_enums.h \
	-enumInputFile ${BOARD_DIR}/board_enums.h \
  -c_defines        controllers/generated/rusefi_generated_${SHORT_BOARD_NAME}.h \
  -c_destination    controllers/generated/engine_configuration_generated_structures_${SHORT_BOARD_NAME}.h

[ $? -eq 0 ] || { echo "ERROR generating TunerStudio config for ${BOARD_DIR}"; exit 1; }

echo "$SCRIPT_NAME: Happy ${SHORT_BOARD_NAME}!"
exit 0
