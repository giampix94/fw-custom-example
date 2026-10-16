/**
 * @file        ms5611.cpp
 * @brief       Driver MS5611-01BA03 (baro I2C bit-bang) per ECUstm32
 *
 * File self-contained (classe + init) per l'overlay board del fork, cosi'
 * non serve toccare il submodule ext/rusefi ne' un header a parte.
 *
 * Aggancio: board_configuration.cpp fa
 *     void initMs5611Baro();                       // forward declaration
 *     custom_board_InitHardware = initMs5611Baro;  // in setup_custom_board_overrides()
 *
 * Bus: istanzia direttamente un BitbangI2c (solo GPIO/PAL), NON usa
 * getI2cBus(scl,sda) che e' solo una ricerca di bus gia' creati da
 * initI2cModules() - che gira dopo questo hook.
 *
 * Formule/comandi dal datasheet MS5611-01BA03 (DA5611-01BA03_011, 26.10.2012).
 * [DS pag.N] = pagina. Schema sensore modellato su hw_layer/sensors/lps25.cpp.
 *
 * HW ECUstm32 proto: SCL=PF0, SDA=PF1 (netlist invertita vs I2C HW -> forza il
 * bit-bang). Indirizzo provato automaticamente 0x76 (CSB->VCC) e 0x77 (CSB->GND).
 *
 * Filtro anti-glitch bit-bang (2026-10-17):
 * il bus bit-bang usa un loop di nop tarato per ~320kHz su un F4 @ 168MHz
 * [ext/rusefi/firmware/hw_layer/drivers/i2c/i2c_bb.cpp, waitQuarterBit()]; sulla
 * nostra F767 @ 216MHz lo stesso loop gira piu' veloce (~411kHz stimato), sopra
 * il limite 400kHz del datasheet MS5611, e in piu' il bit-bang non ha nessuna
 * protezione da interruzioni (nessuna sezione critica in i2c_bb.cpp). Le letture
 * D1/D2 non hanno un CRC per-campione (solo la PROM ce l'ha), quindi un singolo
 * bit corrotto puo' produrre un valore sbagliato ma comunque dentro la finestra
 * di plausibilita' 50-120 kPa, e passare come valido per un ciclo.
 * MAX_STEP_KPA/CONFIRM_SAMPLES sono una soglia di progetto (NON da datasheet):
 * un singolo campione che si scosta troppo dall'ultimo buono viene scartato (si
 * ripubblica l'ultimo valore buono), a meno che lo scostamento non si confermi
 * su CONFIRM_SAMPLES campioni consecutivi concordi (allora e' un cambio vero,
 * es. variazione di quota, non un glitch del bus).
 */

#include "pch.h"
#include "board_overrides.h"
#include "i2c.h"
#include "i2c_bb.h"

#if EFI_PROD_CODE
#include "periodic_thread_controller.h"
#endif

// --- Comandi [DS pag.9, Fig.4] -------------------------------------------
static constexpr uint8_t CMD_RESET      = 0x1E;
static constexpr uint8_t CMD_CONVERT_D1 = 0x48; // pressione, OSR = 4096
static constexpr uint8_t CMD_CONVERT_D2 = 0x58; // temperatura, OSR = 4096
static constexpr uint8_t CMD_ADC_READ   = 0x00;
static constexpr uint8_t CMD_PROM_READ  = 0xA0; // base; |(addr<<1) per la word addr

// OSR=4096: conversione max 9.04 ms [DS pag.3] -> attesa 10 ms
static constexpr int CONVERSION_WAIT_MS = 10;

// Filtro anti-glitch bit-bang: soglia di progetto, vedi nota in testa al file.
static constexpr float MAX_STEP_KPA = 3.0f;
static constexpr int CONFIRM_SAMPLES = 3;

// Bus bit-bang dedicato al baro (self-contained, solo GPIO).
static BitbangI2c ms5611bus;

class Ms5611 : public StoredValueSensor {
public:
	Ms5611() : StoredValueSensor(SensorType::BarometricPressure, MS2NT(1000)) { }

	bool init(brain_pin_e scl, brain_pin_e sda) {
		// crea il bus bit-bang su quei pin
		if (!ms5611bus.init(scl, sda, I2C_SPEED_400K)) {
			efiPrintf("MS5611: init bus bit-bang fallita");
			return false;
		}
		m_i2c = &ms5611bus;

		// prova indirizzo 0x76 (CSB->VCC) poi 0x77 (CSB->GND): chi fa ACK sul reset vince
		const uint8_t addrs[2] = {0x76, 0x77};
		bool acked = false;
		for (int i = 0; i < 2 && !acked; i++) {
			m_addr = addrs[i];
			if (m_i2c->write(m_addr, &CMD_RESET, 1) == MSG_OK) {
				acked = true;
			}
		}
		if (!acked) {
			efiPrintf("MS5611: nessun ACK su 0x76/0x77 - PS a VCC? clock? cablaggio PF0/PF1?");
			return false;
		}
		efiPrintf("MS5611: ACK su 0x%02x", (unsigned)m_addr);

		// dopo il reset ~2.8 ms per il reload della PROM [DS pag.9]
		chThdSleepMilliseconds(5);

		if (!readProm()) {
			efiPrintf("MS5611: lettura PROM fallita");
			return false;
		}
		efiPrintf("MS5611: PROM C1..C6 = %u %u %u %u %u %u",
			(unsigned)m_prom[1], (unsigned)m_prom[2], (unsigned)m_prom[3],
			(unsigned)m_prom[4], (unsigned)m_prom[5], (unsigned)m_prom[6]);

		if (m_prom[1] == 0x0000 || m_prom[1] == 0xFFFF) {
			efiPrintf("MS5611: PROM vuota/0xFFFF - bus o chip KO");
			return false;
		}
		if (!checkCrc()) {
			efiPrintf("MS5611: PROM CRC4 mismatch - clock troppo alto o rumore?");
			return false;
		}

		m_hasInit = true;
		Register();
		efiPrintf("MS5611: OK, sensore baro registrato su 0x%02x", (unsigned)m_addr);
		return true;
	}

	expected<float> readPressureKpa() {
		if (!m_hasInit) {
			return unexpected;
		}

		uint32_t D1 = 0; // pressione grezza
		uint32_t D2 = 0; // temperatura grezza
		if (!convertAndRead(CMD_CONVERT_D2, D2) ||
		    !convertAndRead(CMD_CONVERT_D1, D1)) {
			invalidate();
			return unexpected;
		}

		// --- COMPENSAZIONE MS5611 (1 ordine) [DS pag.7] -------------------
		int32_t C1 = m_prom[1];
		int32_t C2 = m_prom[2];
		int32_t C3 = m_prom[3];
		int32_t C4 = m_prom[4];
		int32_t C5 = m_prom[5];
		int32_t C6 = m_prom[6];

		int32_t dT   = (int32_t)D2 - (C5 << 8);                     // D2 - C5*2^8
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * C6) >> 23);  // 2000 + dT*C6/2^23

		int64_t OFF  = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);  // C2*2^16 + C4*dT/2^7
		int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);  // C1*2^15 + C3*dT/2^8

		// --- COMPENSAZIONE 2 ordine [DS pag.8] ----------------------------
		int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
		if (TEMP < 2000) {
			int64_t d = TEMP - 2000;
			T2    = ((int64_t)dT * dT) >> 31;
			OFF2  = (5 * d * d) >> 1;   // 5*(TEMP-2000)^2 / 2
			SENS2 = (5 * d * d) >> 2;   // 5*(TEMP-2000)^2 / 4
			if (TEMP < -1500) {
				int64_t dv = TEMP + 1500;
				OFF2  += 7 * dv * dv;          // +7*(TEMP+1500)^2
				SENS2 += (11 * dv * dv) >> 1;  // +11*(TEMP+1500)^2 / 2
			}
		}
		TEMP -= (int32_t)T2;
		OFF  -= OFF2;
		SENS -= SENS2;

		// P in 0.01 mbar: (D1*SENS/2^21 - OFF)/2^15 [DS pag.7]
		int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

		// 0.01 mbar -> kPa: P/1000
		float kilopascal = P / 1000.0f;

		// stessi limiti di plausibilita' del driver LPS25: 50-120 kPa
		if (kilopascal > 120.0f || kilopascal < 50.0f) {
			invalidate();
			return unexpected;
		}

		// --- Filtro anti-glitch bit-bang (vedi nota in testa al file) -----
		if (!m_hasLastValid) {
			m_hasLastValid = true;
			m_lastValidKpa = kilopascal;
			m_pendingCount = 0;
		} else if (fabsf(kilopascal - m_lastValidKpa) > MAX_STEP_KPA) {
			// scostamento sospetto rispetto all'ultimo valore buono
			if (m_pendingCount > 0 && fabsf(kilopascal - m_pendingKpa) <= MAX_STEP_KPA) {
				m_pendingCount++;
			} else {
				m_pendingKpa = kilopascal;
				m_pendingCount = 1;
			}

			if (m_pendingCount >= CONFIRM_SAMPLES) {
				// confermato su piu' campioni consecutivi concordi: e' un cambio vero,
				// non un glitch del bus (es. variazione di quota)
				m_lastValidKpa = kilopascal;
				m_pendingCount = 0;
			} else {
				// scarta il campione: ripubblica l'ultimo valore buono (timestamp
				// aggiornato cosi' il sensore non va mai "stale")
				efiPrintf("MS5611: campione scartato %.2f kPa (ultimo buono %.2f kPa, conferma %d/%d)",
					kilopascal, m_lastValidKpa, m_pendingCount, CONFIRM_SAMPLES);
				setValidValue(m_lastValidKpa, getTimeNowNt());
				return m_lastValidKpa;
			}
		} else {
			m_lastValidKpa = kilopascal;
			m_pendingCount = 0;
		}

		setValidValue(m_lastValidKpa, getTimeNowNt());
		return m_lastValidKpa;
	}

	bool hasInit() const { return m_hasInit; }

private:
	i2cBus* m_i2c = nullptr;
	uint8_t m_addr = 0x76;
	bool m_hasInit = false;
	uint16_t m_prom[8] = {0};  // 0=factory, 1..6=C1..C6, 7=serial+CRC

	// Stato del filtro anti-glitch (vedi nota in testa al file).
	bool m_hasLastValid = false;
	float m_lastValidKpa = 0.0f;
	float m_pendingKpa = 0.0f;
	int m_pendingCount = 0;

	bool readProm() {
		for (uint8_t i = 0; i < 8; i++) {
			uint8_t cmd = CMD_PROM_READ | (i << 1);
			uint8_t buf[2];
			if (m_i2c->writeRead(m_addr, &cmd, 1, buf, 2) != MSG_OK) {
				return false;
			}
			m_prom[i] = ((uint16_t)buf[0] << 8) | buf[1];
		}
		return true;
	}

	bool convertAndRead(uint8_t convertCmd, uint32_t& result) {
		if (m_i2c->write(m_addr, &convertCmd, 1) != MSG_OK) {
			return false;
		}
		chThdSleepMilliseconds(CONVERSION_WAIT_MS);
		uint8_t buf[3];
		if (m_i2c->writeRead(m_addr, &CMD_ADC_READ, 1, buf, 3) != MSG_OK) {
			return false;
		}
		result = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
		return result != 0;
	}

	// CRC4 canonico MS56xx (AN520) [DS pag.11]
	bool checkCrc() const {
		uint16_t prom[8];
		for (int i = 0; i < 8; i++) {
			prom[i] = m_prom[i];
		}
		uint16_t crcRead = prom[7] & 0x000F;
		uint16_t nRem = 0x0000;
		prom[7] = 0xFF00 & prom[7];
		for (int cnt = 0; cnt < 16; cnt++) {
			if (cnt % 2 == 1) {
				nRem ^= (uint16_t)(prom[cnt >> 1] & 0x00FF);
			} else {
				nRem ^= (uint16_t)(prom[cnt >> 1] >> 8);
			}
			for (int nBit = 8; nBit > 0; nBit--) {
				if (nRem & 0x8000) {
					nRem = (nRem << 1) ^ 0x3000;
				} else {
					nRem = (nRem << 1);
				}
			}
		}
		nRem = (0x000F & (nRem >> 12));
		return nRem == crcRead;
	}
};

static Ms5611 ms5611device;

#if EFI_PROD_CODE
class Ms5611Thread : public PeriodicController<UTILITY_THREAD_STACK_SIZE> {
public:
	Ms5611Thread() : PeriodicController("MS5611") { }
	void PeriodicTask(efitick_t) override {
		ms5611device.readPressureKpa();
	}
};

static Ms5611Thread ms5611driver;
#endif

// Agganciato a custom_board_InitHardware da board_configuration.cpp.
void initMs5611Baro() {
	// Se c'e' gia' un baro (es. analogico) configurato, non sovrascrivere.
	if (Sensor::hasSensor(SensorType::BarometricPressure)) {
		return;
	}

	// SCL=PF0, SDA=PF1 (netlist invertita vs I2C HW)
	if (ms5611device.init(Gpio::F0, Gpio::F1)) {
#if EFI_PROD_CODE
		ms5611driver.setPeriod(1000 /*ms*/);  // baro ~1 Hz
		ms5611driver.start();
#endif
	}
}
