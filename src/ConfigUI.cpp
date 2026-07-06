#include <avr/wdt.h>

#include "ConfigUI.h"

namespace {
// Dynamically built submenu: one item per schema setting plus Back.
MenuItem radioItems[kSettingCount + 1];

// Radio settings submenu descriptor.
Menu radioMenu = {
    lblRadio,
    radioItems,
    (uint8_t)(kSettingCount + 1)
};

// Main menu layout. Item order controls rendered order.
const MenuItem mainItems[] = {
    { lblRadio,
        MENU_SUBMENU,
        { .submenu = &radioMenu } },

    { lblSave,
        MENU_ACTION,
        { .action = saveConfig } },

    { lblReboot,
        MENU_ACTION,
        { .action = rebootDevice } },
    
    { lblExit, 
        MENU_ACTION, 
        { .action = exitMenu } }
};
} // namespace

// Root menu descriptor exposed to startup code.
Menu mainMenu = {
    lblMain,
    mainItems,
    sizeof(mainItems) / sizeof(MenuItem)
};

// Global menu engine instance.
EmbeddedMenu menu;

// Blocking yes/no prompt helper for serial CLI interactions.
bool confirmYesNo(const __FlashStringHelper* prompt)
{
    Serial.print(prompt);

    while (true) {
        if (!Serial.available()) {
            continue;
        }

        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            continue;
        }

        if (c == 'y' || c == 'Y') {
            Serial.println("yes");
            return true;
        }

        Serial.println("no");
        return false;
    }
}

// Specialized save prompt wrapper.
bool confirmStoreChanges()
{
    return confirmYesNo(F("Store these changes to EEPROM? [y/N]: "));
}

// Menu action: software reboot with confirmation.
void rebootDevice()
{
    Serial.println("Reboot requested.");

    if (!confirmYesNo(F("Reboot device now? [y/N]: "))) {
        Serial.println("Reboot cancelled.");
        menu.redraw();
        return;
    }

    Serial.println("Rebooting...");
    Serial.flush();

    // ATmega4809 direct software reset path, watchdog fallback otherwise.
#if defined(RSTCTRL)
    _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
    while (true) {
    }
#else
    wdt_enable(WDTO_15MS);
    while (true) {
    }
#endif
}

// Menu action: compare against persisted values and conditionally commit.
void saveConfig()
{
    Serial.println("Saving config...");

    const StoredConfig newCfg = buildCurrentConfig();

    StoredConfig oldCfg {};
    const bool hasOldCfg = readValidConfigFromEeprom(oldCfg);

    if (hasOldCfg && sameUserSettings(oldCfg, newCfg)) {
        Serial.println("No config changes. EEPROM not updated.");
        menu.redraw();
        return;
    }

    printPendingChanges(hasOldCfg ? &oldCfg : nullptr, newCfg);

    if (!confirmStoreChanges()) {
        Serial.println("Save cancelled.");
        menu.redraw();
        return;
    }

    if (saveConfigToEeprom(newCfg)) {
        Serial.println("Config saved to EEPROM");
    } else {
        Serial.println("EEPROM save failed");
    }

    menu.redraw();
}

// Build submenu items from schema table.
// This keeps menu editing in sync with persistence and validation logic.
void initMenus()
{
    for (size_t i = 0; i < kSettingCount; ++i) {
        const SettingDescriptor& setting = settingTable[i];
        MenuItem& item = radioItems[i];

        item.label = setting.label;
        item.type = setting.menuType;

        if (setting.menuType == MENU_BOOL) {
            item.data.boolValue = setting.boolValue;
        } else {
            item.data.intData = { setting.intValue, setting.min, setting.max };
        }
    }

    radioItems[kSettingCount].label = lblBack;
    radioItems[kSettingCount].type = MENU_BACK;
    radioItems[kSettingCount].data.submenu = nullptr;
}

// called when exit from menu is requested
extern bool gExitMenuFlag;
void exitMenu()
{
    gExitMenuFlag = true;
    //Serial.println("Exiting menu, returning to normal operation...");
}