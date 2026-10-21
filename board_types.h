#pragma once

// Override di firmware/controllers/algo/board_types.h (submodule, pinnato
// a2d8784fb6f3599044cc0d7ec2c9fcb68912b2f7), che e' letteralmente:
//   #pragma once
//   // placeholder to keep compiler happy in case of no board override
// Questo file viene incluso SEMPRE da rusefi_types.h (submodule) via
// "#include <board_types.h>", e BOARDINC (Makefile riga 200, BOARDINC = $(BOARD_DIR))
// mette la root della board PRIMA di controllers/algo nell'-I path: il compilatore
// trova questo file invece del placeholder, senza nessuna modifica a ext/rusefi e
// senza override di CUSTOM_GEN_CONFIG/gen_config_board.sh (tentativo precedente,
// rimosso: vedi CORREZIONE #5 in board_engine_configuration.txt).
//
// Tipi per i menu a tendina TC-CAN in TunerStudio (custom ... bits, i valori
// "opt1","opt2" sono scritti a mano nella riga custom - questi typedef servono SOLO
// a rendere il tipo del campo struct (tcCanDrivetrain_e tcCanDrivetrainLayout;)
// visibile al compilatore C++, che altrimenti fallisce su
// engine_configuration_generated_structures_ecustm32_proto.h con
// "'tcCanDrivetrain_e' does not name a type" (CORREZIONE #3/#4, run precedenti).

typedef enum __attribute__ ((__packed__)) {
	TC_CAN_DRIVETRAIN_2WD_FRONT = 0,
	TC_CAN_DRIVETRAIN_2WD_REAR = 1,
	TC_CAN_DRIVETRAIN_4WD = 2,
} tcCanDrivetrain_e;

typedef enum __attribute__ ((__packed__)) {
	TC_CAN_METHOD_FIXED_SLIP = 0,
	TC_CAN_METHOD_GEAR_RATIO = 1,
} tcCanMethod_e;
