#pragma once
#include "ConfigSchema.h"

// Backing storage for runtime-configurable settings.  these must be global so they can be referenced by the the setup()
//  and loop() functions, and also by the menu system.
bool gPoll_Inhibit = false;  // global flag to track if CIV Tx is inhibited
int32_t gCIVAddress = 0x94;
int32_t gCIVBaudRate = 115200;
int32_t gPollingInterval = 1000; // default polling interval in milliseconds
int32_t gPollingInactivityTimeout = 30000; // default polling inactivity timeout in milliseconds
bool gExitMenuFlag = false; // flag to signal menu exit and return to normal operation

// Single source of truth for editable settings.
// Add/remove settings here and the UI/store layers adapt automatically.
const SettingDescriptor settingTable[kSettingCount] = {
    { lblTxInhibit, MENU_BOOL, &gPoll_Inhibit, nullptr, 0, 1 },
    { lblCiv, MENU_INT, nullptr, &gCIVAddress, 0, 255 },
    { lblBaud, MENU_INT, nullptr, &gCIVBaudRate, 1200, 115200 },
    { lblPollingInterval, MENU_INT, nullptr, &gPollingInterval, 1000, 30000 },
    { lblPollingInactivityTimeout, MENU_INT, nullptr, &gPollingInactivityTimeout, 1000, 30000 }
};

// Print a null-terminated PROGMEM string.
void printProgmem(PGM_P p)  
{
    char c;
    while ((c = (char)pgm_read_byte(p++))) {
        Serial.write(c);
    }
}

// Read the runtime value for any setting descriptor.
int32_t getSettingValue(const SettingDescriptor& setting)
{
    if (setting.menuType == MENU_BOOL) {
        return *setting.boolValue ? 1 : 0;
    }

    return *setting.intValue;
}

// Write runtime value for any setting descriptor.
void setSettingValue(const SettingDescriptor& setting, int32_t value)
{
    if (setting.menuType == MENU_BOOL) {
        *setting.boolValue = (value != 0);
        return;
    }

    *setting.intValue = value;
}

// Validate a value against descriptor range.
bool settingValueInRange(const SettingDescriptor& setting, int32_t value)
{
    return value >= setting.min && value <= setting.max;
}

// Print integer values in decimal and hex for operator clarity.
void printValueWithHex(int32_t value)
{
    Serial.print(value);
    Serial.print(" (0x");
    Serial.print(static_cast<uint32_t>(value), HEX);
    Serial.print(')');
}

// Print a setting value using type-appropriate representation.
void printSettingValue(const SettingDescriptor& setting, int32_t value)
{
    if (setting.menuType == MENU_BOOL) {
        Serial.print(value ? "ON" : "OFF");
    } else {
        printValueWithHex(value);
    }
}

// Print all current in-memory values at boot.
void printCurrentConfig()
{
    Serial.println("Current config:");

    for (size_t i = 0; i < kSettingCount; ++i) {
        const SettingDescriptor& setting = settingTable[i];

        Serial.print("- ");
        printProgmem(setting.label);
        Serial.print(": ");
        printSettingValue(setting, getSettingValue(setting));
        Serial.println();
    }
}
