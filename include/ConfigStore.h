#pragma once

#include "ConfigSchema.h"

// EEPROM payload format for all settings.
// values[] indices map directly to SettingId / settingTable order.
struct StoredConfig {
    // Signature/version fields for compatibility checks.
    uint32_t magic;
    uint16_t version;
    // Persistent values for each schema setting.
    int32_t values[kSettingCount];
    // Integrity checksum over all prior bytes.
    uint16_t checksum;
};

// Build a fully-populated persistent snapshot from current runtime values.
StoredConfig buildCurrentConfig();

// Read config from EEPROM and validate signature/version/checksum/ranges.
bool readValidConfigFromEeprom(StoredConfig& cfg);

// Compare only user-visible values, ignoring metadata/checksum.
bool sameUserSettings(const StoredConfig& a, const StoredConfig& b);

// Apply stored values to runtime variables bound in settingTable.
void applyConfigValues(const StoredConfig& cfg);

// Write already-built config snapshot to EEPROM.
bool saveConfigToEeprom(const StoredConfig& cfg);

// High-level boot helper: load and apply valid EEPROM config.
bool loadConfigFromEeprom();

// Print line-by-line diff between persisted and current values.
void printPendingChanges(const StoredConfig* oldCfg, const StoredConfig& newCfg);
