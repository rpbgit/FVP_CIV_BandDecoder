#pragma once

#include "ConfigStore.h"

// Global menu engine and root menu descriptor exposed to main.cpp.
extern EmbeddedMenu menu;
extern Menu mainMenu;

// Build dynamic menu items from schema table.
void initMenus();

// Blocking yes/no prompt helper for serial terminal interactions.
bool confirmYesNo(const __FlashStringHelper* prompt);

// Specialized confirmation for saving configuration.
bool confirmStoreChanges();

// Menu actions.
void rebootDevice();
void saveConfig();
void exitMenu();
