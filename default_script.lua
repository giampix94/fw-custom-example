// ============================================================================
// default_script.lua  --  ECUstm32 proto (STM32F767ZIT6)
// Override di board: incluso AL POSTO di
//   ext/rusefi/firmware/controllers/algo/default_script.lua
// tramite  #include <default_script.lua>  risolto su  BOARDINC = $(BOARD_DIR)
//   [rusefi engine_configuration.cpp:635-637 ; board.mk: BOARDINC += $(BOARD_DIR)]
// Va messo nella cartella della board (stessa di board.mk), NON in ext/rusefi.
//
// Installato da setLuaScript() durante applyEngineType()  [settings.cpp:769].
// PRECEDENZA: una Lua page gia' bruciata in flash VINCE sullo script embedded
//   [extra_flash_pages.cpp:34 loadExtraPages -> lua_config_page.cpp:34-43].
//   => per attivarlo la prima volta serve CONFIG ERASE (vedi handoff).
//
// RICHIEDE nel prepend della board:  #define LUA_SCRIPT_SIZE 24000
//   default rusEFI = 8000 (troppo: script = 18k B); Proteus F7 usa 48000
//   [rusefi_config_shared.txt:46 ; prepend_proteus_f7.txt ; fit <128KB
//    lua_config_page.cpp:21 static_assert].
//
// Sorgente: ECU_CAN_PROTOCOL_v1.1.pdf (rev 1.1) + ecu_can_protocol.lua.
// Validato (2026-10-20): CRC-8 vettore "123456789"->0x4B OK; nomi sensore
//   cross-check vs firmware (2 corretti); getTimeSinceStartMs->Timer (corretto);
//   compile + 30x onTick + onCmd (CRC ok/rotto/rc-bloccato) in simulazione OK.
// CORREZIONE (22/10/2026): analisi indipendente (DISALLINEAMENTI_ECU_LUA.md,
//   59 controlli automatici + verifica contro sorgenti rusEFI master) ha
//   trovato 11 difetti critici/medi non coperti dalla validazione del
//   2026-10-20, che controllava solo layout frame/CRC/nomi sensore, non le
//   UNITA' FISICHE restituite da getSensor(). Corretti: A1 (MAP/oilP/fuelP/
//   baro in kPa, non bar: fattore *100 -> *1), A2 (MAF in kg/h, non g/s),
//   A3 (stato motore fisso a "fermo", sopprimeva tutti gli allarmi display),
//   A4 (failsafe non applicato all'avvio: TC a 0 invece che al MASSIMO),
//   B1/B6/B7 (campi non implementati inviati come 0 invece del valore
//   riservato), C1 (byte riservati 0x111 non a 0x00), C3 (arrotondamento).
//   Verificato: sintassi (lua5.3 loadfile), esecuzione su stub con valori
//   di esempio (oilP 400 kPa->4.00 bar, MAF 36 kg/h->10.0 g/s, tcLevel=5
//   pre-0x200, bytes riservati 0x111 a 00). NON testato su hardware reale.
// ============================================================================
	setLuaScript(R"(
--[[ ==========================================================================
     ECUstm32 -- Implementazione ECU_CAN_PROTOCOL rev. 1.1
     Piattaforma: RusEFI su STM32F767ZIT6, script Lua
     --------------------------------------------------------------------------
     STRUTTURA
       1. Configurazione            -- da adattare
       2. Utility di packing        -- non toccare
       3. CRC-8 SAE J1850
       4. Snapshot                  -- garantisce l'atomicita' del cap. 5
       5. Costruzione frame TX
       6. Ricezione 0x200 e failsafe
       7. Scheduler
     --------------------------------------------------------------------------
     ATTENZIONE
     I nomi dei sensori nella tabella SENSOR e le firme delle funzioni RusEFI
     (getSensor, txCan, canRxAdd, setTickRate) VANNO VERIFICATI contro la
     versione di firmware effettivamente impiegata. Sono centralizzati nella
     sezione 1 apposta: se un nome e' diverso, si corregge in un punto solo.
     Uno script che chiama getSensor con un nome inesistente non da' errore:
     restituisce nil, e il campo viaggia come non valido. Il sintomo e' un
     display che mostra "--" su un sensore funzionante.
========================================================================== ]]

------------------------------------------------------------------------------
-- 1. CONFIGURAZIONE
------------------------------------------------------------------------------

local BUS          = 1        -- CAN1, bus abitacolo
local TICK_HZ      = 50       -- 20 ms -> periodo gruppo FAST
local PROTO_VER    = 0x0101   -- rev. 1.1

-- Nomi sensore RusEFI (getSensor). Verificati contro la lista del firmware
-- (firmware/controllers/sensors/auto_generated_sensor.cpp, 96 nomi) il 2026-10-20.
-- NOTA: il match e' case-insensitive (strEqualCaseInsensitive, sensor.cpp), quindi
-- "RPM"/"TPS1"/"MAP" valgono "Rpm"/"Tps1"/"Map". ATTENZIONE: un nome inesistente
-- NON ritorna nil, LANCIA un errore Lua (luaL_error, lua_class_adapters.cpp:78-85)
-- che aborta onTick -> nessun frame. Le righe gia' corrette sotto lo indicano.
local SENSOR = {
  rpm         = "RPM",
  vss         = "VehicleSpeed",
  tps         = "TPS1",
  tps2        = "TPS2",
  aps1        = "AcceleratorPedalPrimary",   -- CORRETTO: era "AcceleratorPedal" (canale grezzo 1, cfr 0x102)
  aps2        = "AcceleratorPedalSecondary", -- CORRETTO: era "AcceleratorPedal2" (INESISTENTE -> errore Lua)
  map         = "MAP",
  oilP        = "OilPressure",
  fuelP       = "FuelPressureInjector",
  clt         = "CLT",
  iat         = "IAT",
  oilT        = "AuxTemp1",
  gearT       = "AuxTemp2",
  fuelT       = "FuelTemperature",
  lambda      = "Lambda1",
  vbat        = "BatteryVoltage",            -- CORRETTO: era "VBatt" (INESISTENTE -> errore Lua)
  fuelLevel   = "FuelLevel",
  maf         = "MAF",
  baro        = "BarometricPressure",
  aux1        = "AuxAnalog1",   -- pressione impianto acqua/alcool
  aux2        = "AuxAnalog2",
  aux3        = "AuxAnalog3",
}

-- Costanti pubblicate in 0x122. Devono coincidere con i limiti reali
-- applicati dalle strategie, altrimenti il display disegna un fondo scala
-- che non esiste.
local CFG = {
  rpmLimiter  = 8000,
  maxAls      = 4,
  maxBoost    = 4,
  maxTc       = 5,
}

-- Soglia di flusso confermato WMI, in bar. Pubblicata in 0x116.
local WMI_PRESS_MIN = 3.00

-- Failsafe: assenza di 0x200 valido oltre questo tempo
local CMD_TIMEOUT_MS = 200

------------------------------------------------------------------------------
-- 2. UTILITY DI PACKING
------------------------------------------------------------------------------

local INVALID_U8, INVALID_U16, INVALID_I16 = 0xFF, 0xFFFF, 0x7FFF

-- Buffer di trasmissione riutilizzato. NON allocare tabelle nuove a ogni
-- tick: su un heap Lua embedded il garbage collector diventa il collo di
-- bottiglia molto prima del bus.
local TX = {0,0,0,0,0,0,0,0}

local function clearTx()
  for i = 1, 8 do TX[i] = 0 end
end

-- Little-endian, indice 1-based come da convenzione Lua.
-- pos = indice del byte basso.
-- CORRETTO (22/10/2026): math.floor(v) tronca (4,35*100=434,9999... -> 434
-- invece di 435). math.floor(v+0.5) arrotonda. [DISALLINEAMENTI_ECU_LUA.md C3]
local function putU16(pos, v)
  if v == nil then v = INVALID_U16 end
  v = math.floor(v + 0.5)
  if v < 0 then v = 0 elseif v > 0xFFFF then v = 0xFFFF end
  TX[pos]     = v % 256
  TX[pos + 1] = math.floor(v / 256) % 256
end

local function putI16(pos, v)
  if v == nil then v = INVALID_I16 end
  v = math.floor(v + 0.5)
  if v < -32768 then v = -32768 elseif v > 32767 then v = 32767 end
  if v < 0 then v = v + 65536 end
  TX[pos]     = v % 256
  TX[pos + 1] = math.floor(v / 256) % 256
end

local function putU8(pos, v)
  if v == nil then v = INVALID_U8 end
  v = math.floor(v + 0.5)
  if v < 0 then v = 0 elseif v > 255 then v = 255 end
  TX[pos] = v
end

local function putI8(pos, v)
  if v == nil then v = 0x7F end
  v = math.floor(v + 0.5)
  if v < -128 then v = -128 elseif v > 127 then v = 127 end
  if v < 0 then v = v + 256 end
  TX[pos] = v
end

-- Scala un valore fisico in intero, propagando nil come "non valido".
local function scale(v, factor)
  if v == nil then return nil end
  return v * factor
end

local function send(id)
  txCan(BUS, id, 0, TX)
end

------------------------------------------------------------------------------
-- 3. CRC-8 SAE J1850  (poly 0x1D, init 0xFF, xorout 0xFF)
------------------------------------------------------------------------------
-- Tabella precalcolata al caricamento dello script: 256 iterazioni una volta
-- sola, contro 56 operazioni di bit a ogni frame. Su un tick a 50 Hz con due
-- frame protetti la differenza e' misurabile.

local CRC_TBL = {}
do
  for i = 0, 255 do
    local c = i
    for _ = 1, 8 do
      if c >= 128 then c = ((c * 2) % 256) ~ 0x1D else c = (c * 2) % 256 end
    end
    CRC_TBL[i] = c
  end
end

local function crc8(t, len)
  local crc = 0xFF
  for i = 1, len do
    crc = CRC_TBL[(crc ~ t[i]) % 256]
  end
  return crc ~ 0xFF
end

------------------------------------------------------------------------------
-- 4. SNAPSHOT
------------------------------------------------------------------------------
-- Requisito del capitolo 5: i sei frame FAST devono nascere da una lettura
-- unica delle variabili. Rileggere getSensor mentre si compongono i frame
-- significa che il regime del frame 0x100 e la pressione del frame 0x101
-- appartengono a istanti diversi, e il consumatore lo ignora.

local S = {}    -- riutilizzata, mai riallocata

local function snapshot()
  S.rpm    = getSensor(SENSOR.rpm)
  S.vss    = getSensor(SENSOR.vss)
  S.tps    = getSensor(SENSOR.tps)
  S.tps2   = getSensor(SENSOR.tps2)
  S.aps1   = getSensor(SENSOR.aps1)
  S.aps2   = getSensor(SENSOR.aps2)
  S.map    = getSensor(SENSOR.map)
  S.oilP   = getSensor(SENSOR.oilP)
  S.fuelP  = getSensor(SENSOR.fuelP)
  S.baro   = getSensor(SENSOR.baro)
  S.lambda = getSensor(SENSOR.lambda)

  -- Grandezze derivate: calcolate QUI, una volta sola, mai nei consumatori.
  if S.map ~= nil and S.baro ~= nil then
    S.boost = S.map - S.baro
  else
    S.boost = nil
  end

  -- Delta benzina: saturato a zero se negativo (cap. 5, campo u16).
  -- La condizione anomala e' segnalata da 0x115 byte 7 bit 1.
  if S.fuelP ~= nil and S.map ~= nil then
    S.dFuel = S.fuelP - S.map
    if S.dFuel < 0 then S.dFuel = 0 end
  else
    S.dFuel = nil
  end

  -- CORRETTO (22/10/2026): stato motore era fisso a 0 ("fermo"), sopprimendo
  -- tutti gli allarmi a valle (display V2.20.0: OIL-P/BOOST-P/FUEL-P/AFR/
  -- KNOCK/EGT-T). Soglie sui giri, proposta da D5 -- da sostituire con lo
  -- stato reale rusEFI quando disponibile. [DISALLINEAMENTI_ECU_LUA.md A3]
  S.gear        = nil
  if S.rpm == nil or S.rpm < 50 then S.engineState = 0
  elseif S.rpm < 600 then S.engineState = 1
  else S.engineState = 2 end
end

------------------------------------------------------------------------------
-- 5. COSTRUZIONE FRAME
------------------------------------------------------------------------------

local rcFast, rcMedium, rcSlow, rcAct = 0, 0, 0, 0

-- ---------- 0x100 ENGINE_CORE ----------
local function tx100()
  clearTx()
  putU16(1, S.rpm)
  putU16(3, scale(S.vss, 10))
  putU8 (5, scale(S.tps, 2))
  putI8 (6, S.gear)
  putU8 (7, S.engineState)
  putU8 (8, rcFast)
  send(0x100)
end

-- ---------- 0x101 PRESSURES ----------
-- CORRETTO (22/10/2026): getSensor per Map/OilPressure/FuelPressureInjector
-- restituisce kPa (rusEFI master: init_map.cpp getMapCfg, es. MT_GM_1_BAR
-- {0,10kpa,5,105kpa}; sensor_type.h, FuelPressureLow/High "// in kPa"; stessa
-- convenzione per OilPressure via initFluidPressure()). Il campo e' in
-- 0,01 bar (specifica App.B), e 1 kPa = 0,01 bar -> fattore 1, non 100.
-- Prima: *100 -> valori 100x (es. 4 bar letti 400 bar). [DISALLINEAMENTI_ECU_LUA.md A1]
local function tx101()
  clearTx()
  putU16(1, scale(S.oilP,  1))
  putI16(3, scale(S.boost, 1))
  putU16(5, scale(S.dFuel, 1))
  putU16(7, scale(S.map,   1))
  send(0x101)
end

-- ---------- 0x102 THROTTLE ----------
local function tx102()
  clearTx()
  putU8(1, scale(S.aps1, 2))
  putU8(2, scale(S.aps2, 2))
  putU8(3, scale(S.tps,  2))
  putU8(4, scale(S.tps2, 2))
  -- TODO: target e duty DBW -- verificare l'accesso nella versione in uso
  putU8(5, nil)
  putI8(6, 0)
  send(0x102)
end

-- ---------- 0x103 COMBUSTION ----------
local function tx103()
  clearTx()
  putI16(1, nil)                          -- TODO anticipo
  putU16(3, scale(S.lambda, 1000))
  putU16(5, nil)                          -- TODO lambda target
  putU8 (7, nil)                          -- TODO duty iniettori
  -- CORRETTO (22/10/2026): era 0 (valore valido, verde su pagina KNOCK del
  -- display). Nessun sensore collegato -> valore riservato u8.
  -- [DISALLINEAMENTI_ECU_LUA.md B1]
  putU8 (8, nil)                          -- TODO livello detonazione
  send(0x103)
end

-- ---------- 0x104 ENGINE_FLAGS ----------
local function tx104()
  clearTx()
  -- TODO: popolare i bit dalle strategie. Vedi nota finale sulle protezioni.
  TX[1] = 0
  TX[2] = 0
  putU16(3, 0)                            -- riduzione coppia TC, 0,1 %
  putU8 (5, alsLevel   or 0)
  putU8 (6, boostLevel or 0)
  putU8 (7, tcLevel    or 0)
  putU8 (8, rcFast)
  send(0x104)
end

-- ---------- 0x105 WHEEL_SPEEDS ----------
local function tx105()
  clearTx()
  -- TODO: quattro ruote. Se non cablate, restano non valide -- corretto.
  putU16(1, nil); putU16(3, nil); putU16(5, nil); putU16(7, nil)
  send(0x105)
end

-- ---------- 0x110 TEMPS_1 ----------
local function tx110()
  clearTx()
  putI16(1, scale(getSensor(SENSOR.clt),   10))
  putI16(3, scale(getSensor(SENSOR.iat),   10))
  putI16(5, scale(getSensor(SENSOR.oilT),  10))
  putI16(7, scale(getSensor(SENSOR.fuelT), 10))
  send(0x110)
end

-- ---------- 0x111 TEMPS_2 (porta il contatore del gruppo MEDIUM) ----------
local function tx111()
  clearTx()
  putI16(1, scale(getSensor(SENSOR.gearT), 10))
  putI16(3, nil)                          -- IAT2
  -- CORRETTO (22/10/2026): byte 4-5 sono "riservato" (cap.12 regola 2: deve
  -- essere 0x00). putI16(nil) scriveva il marcatore non valido 0x7FFF (FF 7F).
  -- Rimossa: clearTx() lascia questi byte a 0x00 come richiesto.
  -- [DISALLINEAMENTI_ECU_LUA.md C1]
  putU8 (7, 0)                            -- riservato
  putU8 (8, rcMedium)                     -- rev. 1.1
  send(0x111)
end

-- ---------- 0x112 EGT ----------
local function tx112()
  clearTx()
  -- TODO: 4x MAX31856 su SPI2, CS su PG10-PG13
  putU16(1, nil); putU16(3, nil); putU16(5, nil); putU16(7, nil)
  send(0x112)
end

-- ---------- 0x113 ANALOG_MISC ----------
-- CORRETTO (22/10/2026): MAF e barometrica.
-- MAF: getSensor(SENSOR.maf) restituisce kg/h (rusEFI master, init_maf.cpp:
-- "This function converts volts -> kg/h"), non g/s. Campo in 0,1 g/s
-- (specifica App.B) -> fattore kg/h*1000/3600*10 = kg/h*(10*1000/3600).
-- Prima: fattore 10, trattava kg/h come g/s (36 kg/h trasmesso come 36,0 g/s
-- invece di 10,0 g/s). [DISALLINEAMENTI_ECU_LUA.md A2]
-- Barometrica: stessa unita' kPa di MAP (init_map.cpp, baroConverter usa lo
-- stesso configureMapFunction/getMapCfg di mapConverter) -> fattore 1, non
-- 100. [DISALLINEAMENTI_ECU_LUA.md A1]
local function tx113()
  clearTx()
  putU16(1, scale(getSensor(SENSOR.vbat), 100))
  putU8 (3, scale(getSensor(SENSOR.fuelLevel), 2))
  putU8 (4, 0)
  putU16(5, scale(getSensor(SENSOR.maf), 10 * 1000 / 3600))
  putU16(7, scale(S.baro, 1))
  send(0x113)
end

-- ---------- 0x114 SPARE_INPUTS ----------
local function tx114()
  clearTx()
  putU16(1, scale(getSensor(SENSOR.aux1), 1000))   -- V -> mV
  putU16(3, scale(getSensor(SENSOR.aux2), 1000))
  putU16(5, scale(getSensor(SENSOR.aux3), 1000))
  putU8 (7, nil)
  putU8 (8, nil)
  send(0x114)
end

-- ---------- 0x115 SENSOR_VALIDITY ----------
-- getSensor restituisce nil quando il sensore non e' valido: e' esattamente
-- il meccanismo che serve. La conferma temporale del cap. 9.2 va aggiunta
-- sopra questo livello, non e' implementata in questo scheletro.
local function bit(cond, n) if cond then return n else return 0 end end

local function tx115()
  clearTx()
  TX[1] = bit(S.map    == nil, 0x04)
        + bit(S.tps    == nil, 0x08)
        + bit(S.tps2   == nil, 0x10)
        + bit(S.aps1   == nil, 0x20)
        + bit(S.aps2   == nil, 0x40)
  TX[2] = bit(S.oilP   == nil, 0x01)
        + bit(S.fuelP  == nil, 0x02)
        + bit(S.baro   == nil, 0x04)
  TX[3] = bit(getSensor(SENSOR.clt)  == nil, 0x01)
        + bit(getSensor(SENSOR.iat)  == nil, 0x02)
        + bit(getSensor(SENSOR.oilT) == nil, 0x08)
  TX[5] = bit(S.lambda == nil, 0x01)
  -- TODO: byte 4 EGT, byte 6 velocita', byte 7 diagnostica derivata
  send(0x115)
end

-- ---------- 0x116 WMI_STATE ----------
-- CORRETTO (22/10/2026): la curva del trasduttore pressione WMI non esiste
-- ancora (TODO sotto); il valore trasmesso era la tensione grezza (V) letta
-- come se fosse gia' in bar*100 (2,5 V -> "2,50 bar" invece di una curva
-- reale). Sinche' la curva non c'e', il campo deve essere non valido: la
-- tensione grezza resta disponibile in 0x114. [DISALLINEAMENTI_ECU_LUA.md A5]
local function tx116()
  clearTx()
  local p = getSensor(SENSOR.aux1)        -- TODO: applicare la curva reale
  putU16(1, nil)
  putU16(3, WMI_PRESS_MIN * 100)
  putU8 (5, nil)                          -- livello serbatoio
  putU8 (6, 0)                            -- duty comandato
  send(0x116)
end

-- ---------- 0x120 SESSION_PEAKS ----------
local pkRpm, pkBoost, pkEgt, pkClt = 0, 0, 0, 0

local function updatePeaks()
  if S.rpm   ~= nil and S.rpm   > pkRpm   then pkRpm   = S.rpm   end
  if S.boost ~= nil and S.boost > pkBoost then pkBoost = S.boost end
  local c = getSensor(SENSOR.clt)
  if c ~= nil and c > pkClt then pkClt = c end
end

-- CORRETTO (22/10/2026): pkBoost e' in kPa (stessa correzione A1, fattore 1
-- non 100); pkEgt non e' mai aggiornato (EGT non implementato, 0x112) e
-- viaggiava come 0 -- ora marcatore non valido u16. [DISALLINEAMENTI_ECU_LUA.md A1/B6]
local function tx120()
  clearTx()
  putU16(1, pkRpm)
  putI16(3, pkBoost * 1)
  putU16(5, nil)
  putI16(7, pkClt * 10)
  send(0x120)
end

-- ---------- 0x121 ECU_HEARTBEAT (porta il contatore del gruppo SLOW) ------
local tickCount, uptimeS = 0, 0

local function tx121()
  clearTx()
  putU16(1, uptimeS)
  putU16(3, PROTO_VER)
  putU8 (5, 0)                            -- TODO TEC
  putU8 (6, 0)                            -- TODO REC
  putU8 (7, 0)                            -- TODO stato controller
  putU8 (8, rcSlow)
  send(0x121)
end

-- ---------- 0x122 ECU_CONFIG ----------
local function tx122()
  clearTx()
  putU16(1, CFG.rpmLimiter)
  putU8 (3, CFG.maxAls)
  putU8 (4, CFG.maxBoost)
  putU8 (5, CFG.maxTc)
  putU8 (6, 0)
  -- CORRETTO (22/10/2026): era 0 (valore valido = "obiettivo 0,00 bar").
  -- [DISALLINEAMENTI_ECU_LUA.md B7]
  putI16(7, nil)                          -- TODO obiettivo boost corrente
  send(0x122)
end

-- ---------- 0x210 ACTUATOR_STATE ----------
local function tx210()
  clearTx()
  -- TODO: popolare da stato reale degli attuatori
  TX[1] = 0                               -- stato applicato
  TX[2] = 0                               -- cause di inibizione
  TX[3] = 0                               -- stato WMI
  TX[4] = 0                               -- duty WMI
  TX[7] = rcAct
  TX[8] = crc8(TX, 7)
  send(0x210)
end

------------------------------------------------------------------------------
-- 6. RICEZIONE 0x200 E FAILSAFE
------------------------------------------------------------------------------

alsLevel, boostLevel, tcLevel = 0, 0, 0    -- globali: lette da tx104
local cmdRace, cmdFan, cmdGearPump, cmdWmi, cmdTcOn = false,false,false,false,false

-- CORRETTO: getTimeSinceStartMs() NON esiste tra gli hook Lua di rusEFI
-- (verificato in lua_hooks.cpp il 2026-10-20). Il clock di comunicazione usa
-- la classe Timer, disponibile e usata negli esempi ufficiali per i timeout
-- di comunicazione (es. honda-bcm.txt). getElapsedSeconds()*1000 -> ms.
local cmdTimer    = Timer.new()
local lastRc      = -1
local rcStuck     = 0
local inFailsafe  = true

local function applyFailsafe()
  inFailsafe  = true
  cmdRace     = false
  cmdFan      = false
  cmdGearPump = false
  cmdWmi      = false
  cmdTcOn     = true
  alsLevel    = 0
  boostLevel  = 0
  tcLevel     = CFG.maxTc      -- cap. 8.1: il TC ripiega al MASSIMO, non a 0
end

-- CORRETTO (22/10/2026): prima dell'arrivo del primo 0x200 valido, i livelli
-- restavano a 0/0/0 (tcLevel incluso) invece che nello stato di failsafe
-- richiesto dalla specifica (TC al massimo). applyFailsafe() va invocata
-- subito, non solo al timeout. [DISALLINEAMENTI_ECU_LUA.md A4]
applyFailsafe()   -- stato sicuro fino al primo 0x200 valido

local function clamp(v, hi)
  if v == nil or v < 0 then return 0 end
  if v > hi then return hi end
  return v
end

local function onCmd(bus, id, dlc, data)
  if dlc < 8 then return end
  if crc8(data, 7) ~= data[8] then return end          -- CRC errato: scartato

  local rc = data[7] % 16
  if rc == lastRc then
    rcStuck = rcStuck + 1
    if rcStuck >= 3 then applyFailsafe() end
    return
  end
  lastRc, rcStuck = rc, 0

  -- Bit 4 del byte 5: nodo in avvio, stati non consolidati -> ignorare
  if (data[6] // 16) % 2 == 1 then
    applyFailsafe()
    return
  end

  local b = data[1]
  cmdRace     = (b % 2) == 1
  cmdFan      = ((b // 2)  % 2) == 1
  cmdGearPump = ((b // 4)  % 2) == 1
  cmdWmi      = ((b // 8)  % 2) == 1
  cmdTcOn     = ((b // 16) % 2) == 1

  alsLevel   = clamp(data[2], CFG.maxAls)
  boostLevel = clamp(data[3], CFG.maxBoost)
  tcLevel    = clamp(data[4], CFG.maxTc)

  cmdTimer:reset()                                     -- CORRETTO: era getTimeSinceStartMs()
  inFailsafe = false
end

canRxAdd(BUS, 0x200, onCmd)

------------------------------------------------------------------------------
-- 7. SCHEDULER
------------------------------------------------------------------------------
-- Un tick = 20 ms. MEDIUM ogni 5 tick, SLOW ogni 25.
-- I frame MEDIUM sono distribuiti sulle 5 fasi: concentrarli sullo stesso
-- tick creerebbe un burst di 14 frame contro 8 di regime.

setTickRate(TICK_HZ)

function onTick()
  tickCount = tickCount + 1

  -- (a) istantanea unica -> atomicita' del gruppo FAST
  snapshot()
  updatePeaks()

  -- (b) failsafe su timeout comandi
  -- CORRETTO: era (getTimeSinceStartMs() - lastCmdMs). getElapsedSeconds()*1000 = ms
  -- dall'ultimo 0x200 valido (cmdTimer:reset() in onCmd).
  if not inFailsafe and (cmdTimer:getElapsedSeconds() * 1000) > CMD_TIMEOUT_MS then
    applyFailsafe()
  end

  -- (c) gruppo FAST: ogni tick
  tx100(); tx101(); tx102(); tx103(); tx104(); tx105()
  tx210()
  rcFast = (rcFast + 1) % 256
  rcAct  = (rcAct  + 1) % 256

  -- (d) gruppo MEDIUM: sfalsato sulle 5 fasi
  local phase = tickCount % 5
  if     phase == 0 then tx110(); tx111(); rcMedium = (rcMedium + 1) % 256
  elseif phase == 1 then tx112(); tx113()
  elseif phase == 2 then tx114(); tx115()
  elseif phase == 3 then tx116()
  end

  -- (e) gruppo SLOW: ogni 25 tick, su una fase libera
  if tickCount % 25 == 4 then
    uptimeS = tickCount // TICK_HZ
    tx120(); tx121(); tx122()
    rcSlow = (rcSlow + 1) % 256
  end
end

--[[ ==========================================================================
     NOTE DI MESSA IN OPERA

     MISURA DEL TICK RATE REALE (chiude il punto aperto del cap. 13)
     Il campo uptime di 0x121 e' calcolato come tickCount / TICK_HZ, cioe'
     assume che lo scheduler rispetti la frequenza richiesta. Confrontandolo
     con un cronometro esterno si misura direttamente lo scarto: se dopo
     600 s reali il campo indica 480, lo scheduler gira a 40 Hz e non a 50,
     e il gruppo FAST ha periodo 25 ms. In quel caso vanno aggiornati sia il
     documento sia i timeout di tutti i nodi.

     PROTEZIONI: NON SONO GUASTI
     I bit del byte 0 di 0x104 (limitatore, cut, TC in intervento) descrivono
     funzionamento normale. Non vanno mai mescolati con 0x115. Questo script
     li tiene in frame separati apposta.

     GARBAGE COLLECTOR
     TX e S sono allocate una volta e riusate. Ogni tabella creata dentro
     onTick diventa spazzatura 50 volte al secondo: su un heap Lua embedded
     e' il primo motivo per cui uno script perde tick.

     BRING-UP
     Verificare in quest'ordine, non tutto insieme:
       1. Solo 0x121 attivo. Cattura con adattatore USB-CAN: periodo 500 ms,
          contatore che incrementa. Se questo non arriva, non serve procedere.
       2. Aggiungere 0x100 e verificare RPM contro TunerStudio.
       3. Gruppo FAST completo. Controllare che il tick non slitti.
       4. Gruppi MEDIUM e SLOW.
       5. Ricezione 0x200 con un frame iniettato a mano, CRC corretto.
       6. Ricezione 0x200 con CRC volutamente errato: deve essere scartato.
       7. Interruzione della trasmissione: il failsafe deve scattare a 200 ms
          e il TC portarsi al MASSIMO.
     Il punto 6 e il punto 7 sono quelli che di solito non vengono provati,
     e sono gli unici due che contano davvero.
========================================================================== ]]
)");
