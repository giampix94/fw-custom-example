#pragma once

// Tipi C per i menu a tendina TC-CAN in TunerStudio (custom ... bits richiede che il tipo
// esista già come typedef C, vedi CORREZIONE #3/#4 in board_engine_configuration.txt).
// Iniettato nella generazione del config via CUSTOM_GEN_CONFIG (board.mk) ->
// gen_config_custom.sh -enumInputFile, SENZA toccare ext/rusefi (gen_config_board.sh del
// submodule ha la sua lista -enumInputFile cablata e non e' parametrizzabile da fuori).
// Verificato: rusefi_config.mk (submodule, pinnato a2d8784fb6f3599044cc0d7ec2c9fcb68912b2f7)
// espone esattamente questo hook:
//   ifneq (,$(CUSTOM_GEN_CONFIG))
//       bash $(BOARD_DIR)/$(CUSTOM_GEN_CONFIG)
//   else
//       bash gen_config_board.sh $(BOARD_DIR) $(SHORT_BOARD_NAME)
//   endif

typedef enum __attribute__ ((__packed__)) {
	TC_CAN_DRIVETRAIN_2WD_FRONT = 0,
	TC_CAN_DRIVETRAIN_2WD_REAR = 1,
	TC_CAN_DRIVETRAIN_4WD = 2,
} tcCanDrivetrain_e;

typedef enum __attribute__ ((__packed__)) {
	TC_CAN_METHOD_FIXED_SLIP = 0,
	TC_CAN_METHOD_GEAR_RATIO = 1,
} tcCanMethod_e;
