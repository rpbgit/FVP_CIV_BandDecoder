#ifndef NANO_EVERY_PORTS_H
#define NANO_EVERY_PORTS_H

#include <avr/io.h>

// -----------------------------
// VPORTA  → D0–D3
// -----------------------------
#define MASK_D0D3        0x0F  // PA0–PA3
#define MASK_D0          (1 << 1)  // RX
#define MASK_D1          (1 << 0)  // TX
#define MASK_D2          (1 << 2)
#define MASK_D3          (1 << 3)

// -----------------------------
// VPORTB  → D4, D11–D13
// -----------------------------
#define MASK_D4_D11_D13  0x0F  // PB0–PB3
#define MASK_D4          (1 << 0)
#define MASK_D13         (1 << 1)
#define MASK_D11         (1 << 2)
#define MASK_D12         (1 << 3)

// -----------------------------
// VPORTC  → D5–D10
// -----------------------------
#define MASK_D5_D10      0x3F  // PC0–PC5
#define MASK_D10         (1 << 0)
#define MASK_D9          (1 << 1)
#define MASK_D8          (1 << 2)
#define MASK_D7          (1 << 3)
#define MASK_D6          (1 << 4)
#define MASK_D5          (1 << 5)

// -----------------------------
// VPORTD  → A0–A7  (includes I2C SDA/SCL on A4/A5)
// -----------------------------
#define MASK_A0_A7       0xFF  // PD0–PD7
#define MASK_A5          (1 << 0)
#define MASK_A4          (1 << 1)
#define MASK_A0          (1 << 2)
#define MASK_A1          (1 << 3)
#define MASK_A2          (1 << 4)
#define MASK_A3          (1 << 5)
#define MASK_A6          (1 << 6)
#define MASK_A7          (1 << 7)

// -----------------------------
// VPORTE  → D14, D15 (Serial1)
// -----------------------------
#define MASK_D14_D15     0x03  // PE0–PE1
#define MASK_D14         (1 << 0)
#define MASK_D15         (1 << 1)

// -----------------------------
// VPORTF → not connected
// -----------------------------

// -----------------------------
// Utility macros
// -----------------------------

// --- Set pins HIGH ---
#define SET_VPORT(port, mask)    ((port).OUTSET = (mask))

// --- Set pins LOW ---
#define CLR_VPORT(port, mask)    ((port).OUTCLR = (mask))

// --- Toggle pins ---
#define TOGGLE_VPORT(port, mask) ((port).OUTTGL = (mask))

// --- Example aliases for convenience ---
#define SET_A0_A3_HIGH()   SET_VPORT(VPORTD, (MASK_A0|MASK_A1|MASK_A2|MASK_A3))
#define CLR_A0_A3_LOW()    CLR_VPORT(VPORTD, (MASK_A0|MASK_A1|MASK_A2|MASK_A3))
#define TOGGLE_A0_A3()     TOGGLE_VPORT(VPORTD, (MASK_A0|MASK_A1|MASK_A2|MASK_A3))

#define SET_D5_D10_HIGH()  SET_VPORT(VPORTC, MASK_D5_D10)
#define CLR_D5_D10_LOW()   CLR_VPORT(VPORTC, MASK_D5_D10)
#define TOGGLE_D5_D10()    TOGGLE_VPORT(VPORTC, MASK_D5_D10)


// #include "NanoEveryPorts.h"

// void setup() {
//   // Configure relevant pins as outputs
//   PORTD.DIRSET = MASK_A0_A7;   // A0–A7 as outputs
//   PORTC.DIRSET = MASK_D5_D10;  // D5–D10 as outputs
// }

// void loop() {
//   TOGGLE_A0_A3();   // Toggle A0–A3 atomically
//   TOGGLE_D5_D10();  // Toggle D5–D10 atomically
//   _delay_ms(200);
// }

#endif // NANO_EVERY_PORTS_H
