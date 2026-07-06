#pragma once

#include <Arduino.h>

// AVR/megaAVR targets provide real PROGMEM helpers.
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#include <avr/pgmspace.h>
#else
// Non-AVR compatibility shims so this header remains portable.
#ifndef PROGMEM
#define PROGMEM
#endif
typedef const char* PGM_P;
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char*)(addr))
#endif
#endif

// Max user input line length (including terminating '\0').
#ifndef EMBEDDED_MENU_INPUT_BUFFER
#define EMBEDDED_MENU_INPUT_BUFFER 32
#endif

// Max nested submenu depth.
#ifndef EMBEDDED_MENU_STACK_DEPTH
#define EMBEDDED_MENU_STACK_DEPTH 8
#endif

// Menu item kinds supported by the embedded text UI.
enum MenuItemType {
    MENU_ACTION,
    MENU_SUBMENU,
    MENU_BOOL,
    MENU_INT,
    MENU_BACK
};

// Forward declaration for pointer members.
struct Menu;

// Integer editor binding + valid range.
struct MenuIntData {
    int32_t* value;
    int32_t min;
    int32_t max;
};

// One displayed entry in a menu screen.
struct MenuItem {
    // Item label stored in program memory when available.
    PGM_P label;
    MenuItemType type;

    // Payload selected by 'type'.
    union {
        void (*action)();
        Menu* submenu;
        bool* boolValue;
        MenuIntData intData;
    } data;
};

// Menu descriptor: title + item table.
struct Menu {
    PGM_P title;
    const MenuItem* items;
    uint8_t itemCount;
};

// Stateful serial-menu engine.
class EmbeddedMenu {
public:
    // Construct with clean runtime state.
    EmbeddedMenu()
        : inputPos(0)
        , currentMenu(nullptr)
        , stackPos(0)
        , editingInt(false)
        , editValue(nullptr)
        , editMin(0)
        , editMax(0)
    {
        inputBuffer[0] = 0;
    }

    // Start the UI from a root menu.
    void begin(Menu* root)
    {
        currentMenu = root;
        redraw();
    }

    // Poll serial port, echo input, and process complete lines.
    void poll()
    {
        while (Serial.available()) {
            char c = (char)Serial.read();

            // Ignore CR (many terminals send CRLF).
            if (c == '\r')
                continue;

            if (c == '\n') {
                // Finalize line and hand off to parser.
                Serial.print("\r\n");

                inputBuffer[inputPos] = 0;
                bool needPrompt = processLine(inputBuffer);

                // Reset input state for next command.
                inputPos = 0;
                inputBuffer[0] = 0;

                // Only print prompt when action path did not redraw.
                if (needPrompt) {
                    prompt();
                }
            } else if (c == 8 || c == 127) {
                // Backspace/Delete handling.
                if (inputPos > 0) {
                    inputPos--;
                    inputBuffer[inputPos] = 0;
                    Serial.print("\b \b");
                }
            } else {
                // Normal character input, bounded by buffer size.
                if (inputPos < (EMBEDDED_MENU_INPUT_BUFFER - 1)) {
                    inputBuffer[inputPos++] = c;
                    inputBuffer[inputPos] = 0;
                    Serial.write(c);
                }
            }
        }
    }

    // Render current menu screen and prompt.
    void redraw()
    {
        clearScreen();

        Serial.print("\r\n");
        printProgmem(currentMenu->title);
        Serial.print("\r\n--------------------\r\n");

        for (uint8_t i = 0; i < currentMenu->itemCount; i++) {
            const MenuItem* item = &currentMenu->items[i];

            // Numbered menu list (1-based index).
            Serial.print((unsigned int)(i + 1));
            Serial.print(") ");
            printProgmem(item->label);

            switch (item->type) {
            case MENU_BOOL:
                // Show ON/OFF value inline.
                Serial.print(" : ");
                Serial.print(*item->data.boolValue ? "ON" : "OFF");
                break;

            case MENU_INT:
                // Show numeric value in decimal and hex.
                Serial.print(" : ");
                Serial.print(*item->data.intData.value);
                Serial.print(" (0x");
                Serial.print((uint32_t)(*item->data.intData.value), HEX);
                Serial.print(")");
                break;

            case MENU_SUBMENU:
                // Visual cue that item enters a submenu.
                Serial.print(" >");
                break;

            default:
                break;
            }

            Serial.print("\r\n");
        }

        prompt();
    }

private:
    // Line input buffer + current write position.
    char inputBuffer[EMBEDDED_MENU_INPUT_BUFFER];
    uint8_t inputPos;

    // Currently displayed menu.
    Menu* currentMenu;

    // Navigation stack for submenu traversal.
    Menu* menuStack[EMBEDDED_MENU_STACK_DEPTH];
    uint8_t stackPos;

    // Integer edit mode state.
    bool editingInt;
    int32_t* editValue;
    int32_t editMin;
    int32_t editMax;

    // Parse and execute one completed input line.
    // Returns true if caller should print prompt immediately.
    bool processLine(char* line)
    {
        if (editingInt) {
            int32_t value;

            // Reject non-numeric text.
            if (!parseIntValue(line, value)) {
                Serial.print("Invalid value\r\n");
                redraw();
                return false;
            }

            // Enforce configured min/max.
            if (value < editMin || value > editMax) {
                Serial.print("Value out of range\r\n");
            } else {
                *editValue = value;
                Serial.print("Updated\r\n");
            }

            // Exit edit mode and return to menu view.
            editingInt = false;
            redraw();
            return false;
        }

        int32_t selection;

        // Invalid/empty selection redraws current screen.
        if (!parseIntValue(line, selection) || selection <= 0) {
            redraw();
            return false;
        }

        return executeSelection((uint8_t)selection);
    }

    // Execute selected menu item.
    // Returns true if caller should print prompt (no redraw path).
    bool executeSelection(uint8_t selection)
    {
        if (selection > currentMenu->itemCount) {
            Serial.print("Invalid selection\r\n");
            return true;
        }

        const MenuItem* item = &currentMenu->items[selection - 1];

        switch (item->type) {
        case MENU_ACTION:
            // Action owns its own output/redraw behavior.
            if (item->data.action)
                item->data.action();
            return false;

        case MENU_SUBMENU:
            // Enter child menu.
            pushMenu(item->data.submenu);
            return false;

        case MENU_BOOL:
            // Toggle bool and redraw immediately.
            *item->data.boolValue = !(*item->data.boolValue);
            redraw();
            return false;

        case MENU_INT:
            // Enter integer editing mode for this bound value.
            editingInt = true;
            editValue = item->data.intData.value;
            editMin = item->data.intData.min;
            editMax = item->data.intData.max;

            Serial.print("Enter value (decimal or 0x hex): ");
            return false;

        case MENU_BACK:
            // Return to previous menu if available.
            popMenu();
            return false;
        }

        return true;
    }

    // Push current menu and switch to submenu.
    void pushMenu(Menu* menu)
    {
        if (stackPos < EMBEDDED_MENU_STACK_DEPTH) {
            menuStack[stackPos++] = currentMenu;
            currentMenu = menu;
            redraw();
        }
    }

    // Pop menu stack and redraw parent menu.
    void popMenu()
    {
        if (stackPos > 0) {
            currentMenu = menuStack[--stackPos];
            redraw();
        }
    }

    // Standard command prompt.
    void prompt()
    {
        Serial.print("\r\nSelection> ");
    }

    // Basic screen separation.
    // ANSI clear sequence kept as optional comment for compatible terminals.
    void clearScreen()
    {
        //Serial.print("\033[2J\033[H");  // ANSI escape code to clear screen and move cursor to home position
        Serial.print("\r\n");
    }

    // Print null-terminated string stored in flash/PROGMEM.
    void printProgmem(PGM_P p)
    {
        char c;

        while ((c = (char)pgm_read_byte(p++))) {
            Serial.write(c);
        }
    }

    // Parse signed integer from line.
    // Supports decimal and 0x/0X prefixed hexadecimal formats.
    bool parseIntValue(char* line, int32_t& value)
    {
        char* end;
        int base = 10;
        char* p = line;

        // Skip leading whitespace.
        while (*p == ' ' || *p == '\t') {
            p++;
        }

        // Skip optional sign for base-detection lookahead.
        if (*p == '+' || *p == '-') {
            p++;
        }

        // Use base-16 if value starts with 0x/0X.
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            base = 16;
        }

        // Parse from original pointer so sign is honored by strtol.
        value = (int32_t)strtol(line, &end, base);

        // Allow trailing whitespace only.
        while (*end == ' ' || *end == '\t') {
            end++;
        }

        // Valid only if something was parsed and no extra garbage remains.
        return end != line && *end == 0;
    }
};

// Define flash-stored string label used by MenuItem/Menu descriptors.
#define MENU_LABEL(name, text) \
    static const char name[] PROGMEM = text
