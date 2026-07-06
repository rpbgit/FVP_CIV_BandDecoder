#include <EEPROM.h>

#include "ConfigStore.h"

namespace {
// EEPROM record metadata.
constexpr uint32_t CONFIG_MAGIC = 0x4D434647; // 'MCFG'
constexpr uint16_t CONFIG_VERSION = 1;
constexpr int CONFIG_EEPROM_ADDR = 0;

// Compute checksum over config payload excluding checksum field itself.
uint16_t calcChecksum(const StoredConfig& cfg)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cfg);
    const size_t n = sizeof(StoredConfig) - sizeof(cfg.checksum);
    uint16_t sum = 0;

    for (size_t i = 0; i < n; ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }

    return sum;
}

// Validate all stored values using schema-defined ranges.
bool isValidRange(const StoredConfig& cfg)
{
    for (size_t i = 0; i < kSettingCount; ++i) {
        if (!settingValueInRange(settingTable[i], cfg.values[i])) {
            return false;
        }
    }

    return true;
}
} // namespace

// Build persistent snapshot from current runtime settings.
StoredConfig buildCurrentConfig()
{
    StoredConfig cfg {};
    cfg.magic = CONFIG_MAGIC;
    cfg.version = CONFIG_VERSION;

    for (size_t i = 0; i < kSettingCount; ++i) {
        cfg.values[i] = getSettingValue(settingTable[i]);
    }

    cfg.checksum = calcChecksum(cfg);
    return cfg;
}

// Read EEPROM record and validate metadata + checksum + ranges.
bool readValidConfigFromEeprom(StoredConfig& cfg)
{
    if ((CONFIG_EEPROM_ADDR + (int)sizeof(StoredConfig)) > EEPROM.length()) {
        return false;
    }

    EEPROM.get(CONFIG_EEPROM_ADDR, cfg);

    if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION) {
        return false;
    }

    if (cfg.checksum != calcChecksum(cfg)) {
        return false;
    }

    if (!isValidRange(cfg)) {
        return false;
    }

    return true;
}

bool sameUserSettings(const StoredConfig& a, const StoredConfig& b)
{
    // Compare only settings payload array for change detection.
    for (size_t i = 0; i < kSettingCount; ++i) {
        if (a.values[i] != b.values[i]) {
            return false;
        }
    }

    return true;
}

// Apply persisted values to runtime variables through schema bindings.
void applyConfigValues(const StoredConfig& cfg)
{
    for (size_t i = 0; i < kSettingCount; ++i) {
        setSettingValue(settingTable[i], cfg.values[i]);
    }
}

// Write validated snapshot to EEPROM address block.
bool saveConfigToEeprom(const StoredConfig& cfg)
{
    if ((CONFIG_EEPROM_ADDR + (int)sizeof(StoredConfig)) > EEPROM.length()) {
        return false;
    }

    EEPROM.put(CONFIG_EEPROM_ADDR, cfg);
    return true;
}

// High-level boot helper: read, validate, and apply.
bool loadConfigFromEeprom()
{
    StoredConfig cfg {};
    if (!readValidConfigFromEeprom(cfg)) {
        return false;
    }

    applyConfigValues(cfg);
    return true;
}

// Print pending changes between previous and current snapshots.
// oldCfg == nullptr means this is the first valid save.
void printPendingChanges(const StoredConfig* oldCfg, const StoredConfig& newCfg)
{
    Serial.println("Pending changes:");

    for (size_t i = 0; i < kSettingCount; ++i) {
        const SettingDescriptor& setting = settingTable[i];

        if (oldCfg != nullptr && oldCfg->values[i] == newCfg.values[i]) {
            continue;
        }

        Serial.print("- ");
        printProgmem(setting.label);
        Serial.print(": ");

        if (oldCfg == nullptr) {
            Serial.print("<unset> -> ");
            printSettingValue(setting, newCfg.values[i]);
            Serial.println();
            continue;
        }

        printSettingValue(setting, oldCfg->values[i]);
        Serial.print(" -> ");
        printSettingValue(setting, newCfg.values[i]);
        Serial.println();
    }
}
