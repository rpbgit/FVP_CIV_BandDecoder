#pragma once

#include <Arduino.h>

#include "Menu.h"

// Stable IDs for settings stored in EEPROM.
// Keep ordering aligned with settingTable for deterministic persistence mapping.
enum SettingId : uint8_t {
    SETTING_TX_INHIBIT = 0,
    SETTING_CIV,
    SETTING_BAUD,
    SETTING_POLLING_INTERVAL,
    SETTING_POLLING_INACTIVITY_TIMEOUT,

    SETTING_COUNT // must always be last, used internally to size the schema table and validate indices
};

// Number of managed settings in the schema table.
constexpr size_t kSettingCount = SETTING_COUNT;

// Shared menu labels.
inline constexpr char lblMain[] PROGMEM = "Main Menu";
inline constexpr char lblReboot[] PROGMEM = "Reboot";
inline constexpr char lblRadio[] PROGMEM = "Radio Settings";
inline constexpr char lblSave[] PROGMEM = "Save Config";
inline constexpr char lblBack[] PROGMEM = "Back";
inline constexpr char lblTxInhibit[] PROGMEM = "Polling Inhibit";
inline constexpr char lblPollingInterval[] PROGMEM = "Polling Interval (ms)";
inline constexpr char lblPollingInactivityTimeout[] PROGMEM = "Polling Inactivity Timeout (ms)";
inline constexpr char lblCiv[] PROGMEM = "CI-V Address";
inline constexpr char lblBaud[] PROGMEM = "CI-V Baudrate";
inline constexpr char lblExit[] PROGMEM = "Exit config, back to normal ops";

// Runtime configuration values.
//extern bool gTxInhibit;???
//extern bool gExitMenuFlag; // flag to signal menu exit and return to normal operation

// Unified schema record used by menu/UI/persistence layers.
struct SettingDescriptor {
    // Display label in PROGMEM.
    PGM_P label;
    // How this setting is edited in the menu.
    MenuItemType menuType;
    // Bound runtime pointer for bool settings.
    bool* boolValue;
    // Bound runtime pointer for integer settings.
    int32_t* intValue;
    // Valid range used for validation and integer menu editing.
    int32_t min;
    int32_t max;
};

// Single source-of-truth schema table (defined in ConfigSchema.cpp).
extern const SettingDescriptor settingTable[kSettingCount];

// Low-level PROGMEM print helper for labels.
void printProgmem(PGM_P p);

// Generic getters/setters for schema-backed values.
int32_t getSettingValue(const SettingDescriptor& setting);
void setSettingValue(const SettingDescriptor& setting, int32_t value);

// Range validation helper for schema values.
bool settingValueInRange(const SettingDescriptor& setting, int32_t value);

// Shared formatting helpers for numeric and polymorphic setting output.
void printValueWithHex(int32_t value);
void printSettingValue(const SettingDescriptor& setting, int32_t value);

// Boot/status dump of current in-memory configuration.
void printCurrentConfig();
