/**
 * ECUstm32 - proto 1 canale ("provaecu")
 * MCU: STM32F767ZIT6, LQFP144
 *
 * FONTE di ogni assegnazione pin:
 *   [NET]  = netlist EasyEDA Sheet_1_2026-08-05.net (stesso set componenti/valori del
 *            BOM_provaecu_2026-07-15.csv, cioe la scheda fabbricata)
 *   [ST]   = STMicroelectronics STM32_open_pin_data, STM32F767ZITx.xml
 *            (posizione pin LQFP144 -> nome porta, funzioni alternative)
 *   [ADC]  = ext/rusefi hw_layer/ports/stm32/stm32_adc_channel_map.h (pin -> EFI_ADC_n)
 *
 * Qui dentro ci sono SOLO fatti hardware della scheda. Tutto quello che dipende
 * dal motore (cilindri, ordine di scoppio, ruota fonica, iniettori, sensori) si
 * imposta in TunerStudio.
 */

#include "pch.h"
#include "board_overrides.h"
#include "ignition_controller.h"
#include "idle_thread.h"
#include <rusefi/timer.h>

void setupBoardHardwareTestOverrides();
void initMs5611Baro();  // definita in ms5611.cpp

// Nessun LED collegato all'MCU sulla proto [NET: U2/U3 sono LED su 12V_MAIN/12V_KEY]
Gpio getCommsLedPin() {
  return Gpio::Unassigned;
}

Gpio getRunningLedPin() {
  return Gpio::Unassigned;
}

Gpio getWarningLedPin() {
  return Gpio::Unassigned;
}

/*
 * Coefficiente partitore per canale ADC.
 * Ingressi 5V (MAP, TPS, WBO2): CN1 -> 10k serie -> pin ADC, 15k pin ADC -> AGND
 *   [NET: R10/R12 su PA5, R11/R13 su PA6, R18/R19 su PC3]  coefficiente = 25/15
 * NTC (CLT, IAT): pull-up 2.49k a VREF_3.3V, 2.2k serie, nessun partitore  coefficiente = 1
 * VBAT (PC5) usa vbattDividerCoeff, non questo [ext/rusefi init/sensor/init_vbatt.cpp].
 */
static float protoGetAnalogInputDividerCoefficient(adc_channel_e hwChannel) {
  switch (hwChannel) {
  case EFI_ADC_2:  // PA2 = IAT
  case EFI_ADC_3:  // PA3 = CLT
    return 1.0f;
  default:
    return (10.0f + 15.0f) / 15.0f;
  }
}

/*
 * Pull-up NTC alimentate da VREF_3.3V (LM1117S-3.3 U4 -> L2) [NET; BOM C126027].
 * Stesso net su VREF+/VDDA via L5: lettura NTC raziometrica.
 */
static float protoGetThermistorSupplyVoltage(adc_channel_e) {
  return 3.3f;
}

static void customBoardDefaultConfiguration() {
  // ---------------- Uscite ----------------

  // INJ1: PD3 (U12-117) -> U8 74AHCT244 -> R41 100R -> Q5 VND5N07 -> CN1-15 [NET]
  engineConfiguration->injectionPins[0] = Gpio::D3;
  engineConfiguration->injectionPinMode = OM_DEFAULT;

  // SPK1: PG0 (U12-56) -> U8 -> SJ2 (smart o IGBT Q4) -> CN1-16, attivo alto [NET]
  engineConfiguration->ignitionPins[0] = Gpio::G0;
  engineConfiguration->ignitionPinMode = OM_DEFAULT;

  // Pompa benzina: PD0 (U12-114) -> R16 1k -> gate Q1 KAO3400 -> CN1-17 [NET]
  engineConfiguration->fuelPumpPin = Gpio::D0;
  engineConfiguration->fuelPumpPinMode = OM_DEFAULT;

  // Main relay: PG14 (U12-129) -> R38 1k -> gate Q3 KAO3400 -> CN1-18 [NET]
  engineConfiguration->mainRelayPin = Gpio::G14;
  engineConfiguration->mainRelayPinMode = OM_DEFAULT;

  // Idle: PB11 (U12-70, TIM2_CH4 [ST]) -> U11 1G125 -> Q6 IRF540N -> CN1-19 [NET]
  engineConfiguration->idle.solenoidPin = Gpio::B11;
  engineConfiguration->idle.solenoidPinMode = OM_DEFAULT;

  // Contagiri: PC6 (U12-96, TIM3_CH1 [ST]) -> Q7 MMBT2222A open collector -> CN1-7 [NET]
  engineConfiguration->tachOutputPin = Gpio::C6;
  engineConfiguration->tachOutputPinMode = OM_DEFAULT;

  // WP1_PWM (PB7, CN1-23) lasciata NON assegnata: uscita generica, si assegna in TS.

  // ---------------- Ingressi ----------------

  // Albero motore VR1: CN1-13/14 -> U6 MAX9926 Mode A2 -> COUT1 -> R31 -> PE9 (U12-60) [NET]
  // PE9 = TIM1_CH1 [ST]
  engineConfiguration->triggerInputPins[0] = Gpio::E9;
  engineConfiguration->triggerInputPins[1] = Gpio::Unassigned;

  // Chiave IGN_SENSE: 12V_KEY -> 47k/10k -> R3 1k -> PG15 (U12-132) [NET]
  // NECESSARIO: VBAT e' letto da 12V_MAIN (dopo il main relay); senza chiave
  // RusEFI non chiuderebbe mai il relay [ignition_controller.cpp, main_relay.cpp].
  engineConfiguration->ignitionKeyDigitalPin = Gpio::G15;
  engineConfiguration->ignitionKeyDigitalPinMode = PI_DEFAULT;

  // Frizione: CN1-22 -> R21 10k -> PE7 (U12-58), pull-up 2.2k a 3.3V [NET]
  // Interruttore verso massa = premuto -> livello basso -> invertito.
  engineConfiguration->clutchDownPin = Gpio::E7;
  engineConfiguration->clutchDownPinMode = PI_INVERTED_DEFAULT;

  // ---------------- Analogici ----------------
  // [ADC]: PA2=ADC_2 PA3=ADC_3 PA5=ADC_5 PA6=ADC_6 PC3=ADC_13 PC5=ADC_15
  engineConfiguration->iat.adcChannel = EFI_ADC_2;          // PA2 <- CN1-9  IAT
  engineConfiguration->clt.adcChannel = EFI_ADC_3;          // PA3 <- CN1-8  CLT
  engineConfiguration->map.sensor.hwChannel = EFI_ADC_5;    // PA5 <- CN1-10 MAP
  engineConfiguration->tps1_1AdcChannel = EFI_ADC_6;        // PA6 <- CN1-11 TPS
  engineConfiguration->afr.hwChannel = EFI_ADC_13;          // PC3 <- CN1-12 WBO2

  // Pull-up NTC 2.49k [NET R8/R14; BOM C705842]
  engineConfiguration->clt.config.bias_resistor = 2490;
  engineConfiguration->iat.config.bias_resistor = 2490;

  // VBAT: 12V_MAIN -> 47k / 10k -> R48 1k -> PC5 [NET]  coeff = (47+10)/10
  engineConfiguration->vbattAdcChannel = EFI_ADC_15;
  engineConfiguration->vbattDividerCoeff = (47.0f + 10.0f) / 10.0f;

  engineConfiguration->analogInputDividerCoefficient = (10.0f + 15.0f) / 15.0f;
  engineConfiguration->adcVcc = 3.3f;

  // ---------------- CAN ----------------
  // PB9 = CAN1_TX (U12-140), PB8 = CAN1_RX (U12-139) [ST] -> U10 SN65HVD230 [NET]
  engineConfiguration->canTxPin = Gpio::B9;
  engineConfiguration->canRxPin = Gpio::B8;

  engineConfiguration->isSdCardEnabled = false;
}

// ======================= TURBO TIMER =======================
// Tiene il motore acceso per turboTimerDurationSec dopo il key-off, per far
// raffreddare il turbo. Campi in board_engine_configuration.txt, UI in
// board_panels.ini / board_menu.ini.
//
// Gira in coda a doPeriodicSlowCallback (dopo MainRelayController), quindi:
//  - il BYPASS del main relay e' deterministico: riscriviamo il pin dopo che
//    il controller del relay lo ha gia' aggiornato [engine.cpp, commento
//    "after modules and board code"];
//  - rusEFI NON taglia iniezione/accensione al key-off (checkShutdown() e'
//    codice morto [engine.cpp]), quindi il motore resta vivo finche' gira.
//
// Criteri di attivazione: valutati sul PICCO raggiunto durante la guida.
// Nessun criterio selezionato -> timer classico (sempre attivo al key-off).
//
// Minimo alzato: loop proporzionale CLAMPATO su luaAdd (l'ingresso che rusEFI
// espone a Lua per aggiungere aria al minimo [idle_thread.cpp]). Il clamp duro
// su luaAdd (max 40%) e' la sicurezza: neanche una lettura RPM errata puo'
// spalancare la valvola. La casella giri in TS si sblocca solo se un'uscita
// minimo e' assegnata.

static bool  ttCountingDown = false;
static bool  ttPrevIgn      = false;
static bool  ttFirst        = true;
static bool  ttWeRaisedIdle = false;
static Timer ttTimer;
static float ttPeakEgt = 0, ttPeakOil = 0, ttPeakClt = 0, ttPeakMap = 0;

static void ttResetPeaks() {
  ttPeakEgt = ttPeakOil = ttPeakClt = ttPeakMap = 0;
}

// Azzera il bump di minimo SOLO se siamo stati noi ad applicarlo, per non
// calpestare un eventuale setIdleAdd fatto da Lua durante il normale uso.
static void ttClearIdle() {
  if (ttWeRaisedIdle) {
    engine->module<IdleController>().unmock().luaAdd = 0;
    ttWeRaisedIdle = false;
  }
}

static void turboTimerSlowCallback() {
  bool ign = isIgnVoltage();
  if (ttFirst) {
    ttPrevIgn = ign;
    ttFirst = false;
  }

  // --- aggiorna i picchi mentre il motore gira davvero ---
  if (engine->rpmCalculator.isRunning()) {
    float v;
    v = Sensor::getOrZero(SensorType::EGT1);           if (v > ttPeakEgt) ttPeakEgt = v;
    v = Sensor::getOrZero(SensorType::OilTemperature); if (v > ttPeakOil) ttPeakOil = v;
    v = Sensor::getOrZero(SensorType::Clt);            if (v > ttPeakClt) ttPeakClt = v;
    v = Sensor::getOrZero(SensorType::Map);            if (v > ttPeakMap) ttPeakMap = v;
  }

  // --- transizione chiave ON -> OFF: decidi se avviare il countdown ---
  if (ttPrevIgn && !ign
      && engineConfiguration->turboTimerEnabled
      && engine->rpmCalculator.isRunning()) {
    bool useEgt   = engineConfiguration->turboTimerUseEgt;
    bool useOil   = engineConfiguration->turboTimerUseOilTemp;
    bool useClt   = engineConfiguration->turboTimerUseClt;
    bool useBoost = engineConfiguration->turboTimerUseBoost;
    bool anyUse   = useEgt || useOil || useClt || useBoost;
    bool trig =
      (useEgt   && ttPeakEgt > engineConfiguration->turboTimerEgtThreshold)     ||
      (useOil   && ttPeakOil > engineConfiguration->turboTimerOilTempThreshold) ||
      (useClt   && ttPeakClt > engineConfiguration->turboTimerCltThreshold)     ||
      (useBoost && ttPeakMap > engineConfiguration->turboTimerBoostThreshold);
    if (!anyUse || trig) {
      ttCountingDown = true;
      ttTimer.reset();
      efiPrintf("TurboTimer: avviato per %d s", engineConfiguration->turboTimerDurationSec);
    }
  }

  // --- transizione chiave OFF -> ON: nuova sessione, azzera i picchi ---
  if (!ttPrevIgn && ign) {
    ttResetPeaks();
  }
  ttPrevIgn = ign;

  if (!ttCountingDown) {
    ttClearIdle();
    return;
  }

  // --- condizioni di stop del countdown ---
  if (ign) {  // chiave tornata su ON
    efiPrintf("TurboTimer: chiave ON, interrompo");
    ttCountingDown = false;
    ttClearIdle();
    return;
  }
  if (engine->rpmCalculator.isStopped()) {  // motore fermo: niente da raffreddare
    efiPrintf("TurboTimer: motore fermo, interrompo");
    ttCountingDown = false;
    ttClearIdle();
    return;
  }
  if (ttTimer.getElapsedSeconds() >= engineConfiguration->turboTimerDurationSec) {
    efiPrintf("TurboTimer: countdown finito, rilascio il relay");
    ttCountingDown = false;
    ttClearIdle();
    return;
  }

  // --- BYPASS MAIN RELAY: tieni ECU e motore alimentati ---
  enginePins.mainRelay.setValue("tt", true);

  // --- minimo alzato (loop proporzionale clampato su luaAdd) ---
  if (engineConfiguration->turboTimerRaiseIdle
      && engineConfiguration->turboTimerIdleRpm > 0) {
    float target = engineConfiguration->turboTimerIdleRpm;
    float rpm = Sensor::getOrZero(SensorType::Rpm);
    if (rpm > 50) {  // lettura RPM plausibile
      auto& add = engine->module<IdleController>().unmock().luaAdd;
      float step = (target - rpm) * 0.01f;      // passo proporzionale
      if (step > 0.5f)  step = 0.5f;            // max +0.5% per ciclo (20 Hz)
      if (step < -0.5f) step = -0.5f;
      add += step;
      if (add < 0)     add = 0;
      if (add > 40.0f) add = 40.0f;             // clamp duro di sicurezza
      ttWeRaisedIdle = true;
    }
  } else {
    ttClearIdle();
  }
}
// ===================== FINE TURBO TIMER =====================

void setup_custom_board_overrides() {
  setupBoardHardwareTestOverrides();
  custom_board_DefaultConfiguration = customBoardDefaultConfiguration;
  custom_board_getAnalogInputDividerCoefficient = protoGetAnalogInputDividerCoefficient;
  custom_board_getThermistorSupplyVoltage = protoGetThermistorSupplyVoltage;
  custom_board_InitHardware = initMs5611Baro;  // baro MS5611 su I2C bit-bang PF0/PF1
  custom_board_periodicSlowCallback = turboTimerSlowCallback;  // turbo timer
}
