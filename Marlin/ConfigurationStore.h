#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"

// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the addresses below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.

#define EEPROM_VERSION "V15"

// EEPROM OFFSET
// offsets start of config from beginning of memory - eeprom has a limited life (c. 10k writes)
// config can be moved if eeprom has burnt out
#define EEPROM_OFFSET 100

/* EEPROM ADDRESSES
 * Fixed offsets in EEPROM memory for each parameter / array of parameters
 * If any of these values are changed, version number should be incremented
 * EEPROM Addresses.xlsx spreadsheet may be used to generate the above (keeping this up to date would be appreciated)
 * For space allowances, refer to "Data Types" on http://arduino.cc/en/Reference/HomePage
 * Unsigned/signed types are of identical size
 * Arrays are [datatype size] x [array size], e.g. float[4] = 16 bytes
 * Quick reference:
 *   - byte, char, bool = 1
 *   - int = 2 ~ 4 (defaults to short on 8bit targets, but on 32bit (ARM) will be long. allow for 4 bytes)
 *   - short = 2
 *   - long = 4
 *   - float = 4
 *   - double = 4 ~ 8 (double precision not used on 8-bit targets - consider using float. allow for 8)
 */
#define EEPROM_ADDR_VER (EEPROM_OFFSET+0) // size=4
#define EEPROM_ADDR_AXIS_STEPS_PER_UNIT (EEPROM_OFFSET+4) // size=16
#define EEPROM_ADDR_MAX_FEEDRATE (EEPROM_OFFSET+20) // size=16
#define EEPROM_ADDR_MAX_ACCELERATION_UNITS_PER_SQ_SECOND (EEPROM_OFFSET+36) // size=16
#define EEPROM_ADDR_ACCELERATION (EEPROM_OFFSET+52) // size=4
#define EEPROM_ADDR_RETRACT_ACCELERATION (EEPROM_OFFSET+56) // size=4
#define EEPROM_ADDR_MINIMUMFEEDRATE (EEPROM_OFFSET+60) // size=4
#define EEPROM_ADDR_MINTRAVELFEEDRATE (EEPROM_OFFSET+64) // size=4
#define EEPROM_ADDR_MINSEGMENTTIME (EEPROM_OFFSET+68) // size=4
#define EEPROM_ADDR_MAX_XY_JERK (EEPROM_OFFSET+72) // size=4
#define EEPROM_ADDR_MAX_Z_JERK (EEPROM_OFFSET+76) // size=4
#define EEPROM_ADDR_MAX_E_JERK (EEPROM_OFFSET+80) // size=4
#define EEPROM_ADDR_ADD_HOMING (EEPROM_OFFSET+84) // size=12
#define EEPROM_ADDR_ENDSTOP_ADJ (EEPROM_OFFSET+96) // size=12
#define EEPROM_ADDR_DELTA_RADIUS (EEPROM_OFFSET+108) // size=4
#define EEPROM_ADDR_DELTA_DIAGONAL_ROD (EEPROM_OFFSET+112) // size=4
#define EEPROM_ADDR_DELTA_SEGMENTS_PER_SECOND (EEPROM_OFFSET+116) // size=4
#define EEPROM_ADDR_PLAPREHEATHOTENDTEMP (EEPROM_OFFSET+120) // size=4
#define EEPROM_ADDR_PLAPREHEATHPBTEMP (EEPROM_OFFSET+124) // size=4
#define EEPROM_ADDR_PLAPREHEATFANSPEED (EEPROM_OFFSET+128) // size=4
#define EEPROM_ADDR_ABSPREHEATHOTENDTEMP (EEPROM_OFFSET+132) // size=4
#define EEPROM_ADDR_ABSPREHEATHPBTEMP (EEPROM_OFFSET+136) // size=4
#define EEPROM_ADDR_ABSPREEATFANSPEED (EEPROM_OFFSET+140) // size=4
#define EEPROM_ADDR_ZPROBE_ZOFFSET (EEPROM_OFFSET+144) // size=4
#define EEPROM_ADDR_KP (EEPROM_OFFSET+148) // size=12
#define EEPROM_ADDR_KI (EEPROM_OFFSET+160) // size=12
#define EEPROM_ADDR_KD (EEPROM_OFFSET+172) // size=12
#define EEPROM_ADDR_KC (EEPROM_OFFSET+184) // size=12
#define EEPROM_ADDR_BEDKP (EEPROM_OFFSET+196) // size=4
#define EEPROM_ADDR_BEDKI (EEPROM_OFFSET+200) // size=4
#define EEPROM_ADDR_BEDKD (EEPROM_OFFSET+204) // size=4
#define EEPROM_ADDR_LCD_CONTRAST (EEPROM_OFFSET+208) // size=2
#define EEPROM_ADDR_AXIS_SCALING (EEPROM_OFFSET+210) // size=12
#define EEPROM_ADDR_AUTORETRACT_ENABLED (EEPROM_OFFSET+222) // size=1
#define EEPROM_ADDR_RETRACT_LENGTH (EEPROM_OFFSET+223) // size=4
#define EEPROM_ADDR_RETRACT_LENGTH_SWAP (EEPROM_OFFSET+227) // size=4
#define EEPROM_ADDR_RETRACT_FEEDRATE (EEPROM_OFFSET+231) // size=4
#define EEPROM_ADDR_RETRACT_ZLIFT (EEPROM_OFFSET+235) // size=4
#define EEPROM_ADDR_RETRACT_RECOVER_LENGTH (EEPROM_OFFSET+239) // size=4
#define EEPROM_ADDR_RETRACT_RECOVER_LENGTH_SWAP (EEPROM_OFFSET+243) // size=4
#define EEPROM_ADDR_RETRACT_RECOVER_FEEDRATE (EEPROM_OFFSET+247) // size=4
#define EEPROM_ADDR_VOLUMETRIC_ENABLED (EEPROM_OFFSET+251) // size=1
#define EEPROM_ADDR_FILAMENT_SIZE (EEPROM_OFFSET+252) // size=12
#define EEPROM_ADDR_HYSTERESIS_CORRECTIONS (EEPROM_OFFSET+264) // size=16
#define EEPROM_ADDR_NEXT_AVAILABLE (EEPROM_OFFSET+280) // size=0

void Config_ResetDefault();

#ifndef DISABLE_M503
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {}
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#endif//CONFIG_STORE_H