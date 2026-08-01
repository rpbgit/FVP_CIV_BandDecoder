#include <Arduino.h>
#pragma once
#include <stdint.h>

// AuxBus byte decoder for Nano Every using interrupt-driven timing.
// A falling-edge GPIO interrupt arms TCB0 one-shot sampling at bit-center points.
// The TCB0 ISR runs a small receive state machine to validate framing and collect data.
// Long timing gaps are split into 16-bit chunks to fit TCB0 compare limits.
// Frame errors enter a 100-bit cooldown before re-arming start-edge detection.
// Internal ISR state and helpers stay file-local in the unnamed namespace.
// Public wrappers expose setup, optional debug loop, and safe byte retrieval.


// ===== User settings =====
constexpr uint8_t RX_PIN = 4; // Arduino D4 = PC6 on Nano Every 4809
constexpr uint32_t BAUD = 998UL; // derived from scope trace to put it in the middle of the bit
constexpr uint8_t TCB_CLK_DIV = 2; // TCB_CLKSEL_CLKDIV2_gc => CLK_PER / 2
constexpr uint32_t TCB_TICK_HZ = F_CPU / TCB_CLK_DIV;
constexpr uint16_t BIT_TICKS = static_cast<uint16_t>((TCB_TICK_HZ + (BAUD / 2UL)) / BAUD);
constexpr uint16_t HALF_BIT_TICKS = BIT_TICKS / 2;
constexpr uint16_t START_LOW_BITS = 80;
constexpr uint32_t START_LOW_TICKS = static_cast<uint32_t>(START_LOW_BITS) * BIT_TICKS;
constexpr uint8_t START_BITS = 5;
// Frame-error cooldown delays re-sync to avoid retriggering on the same noisy burst.
// 100 bit-times is a conservative quiet window before re-arming start-edge detection.
constexpr uint16_t ERROR_WAIT_BITS = 100;
constexpr uint32_t ERROR_WAIT_TICKS = static_cast<uint32_t>(ERROR_WAIT_BITS) * BIT_TICKS;

static_assert(BIT_TICKS > 0, "BIT_TICKS must be non-zero"); // compile-time check to ensure that the calculated BIT_TICKS is valid. If BAUD is too high, this will fail.

// Returns decoded byte [0..255] when available, otherwise -1.
// Consumes the byte (clears ready flag) when one is returned.
int16_t auxBusReadDecodedByte();

// Unnamed namespace gives internal linkage and privacy to AuxBus implementation details in this file.
// This hides ISR state/helpers from other translation units and avoids symbol collisions,
// while keeping only explicit API wrappers globally visible.
namespace {

// ===== Decoder state =====
enum RxState : uint8_t {
    ___DO_NOT_USE,
    IDLE,
    WAIT_FOR_FIRST_START_BIT,
    READING_START_BITS,
    READING_DATA_BITS,
    FRAME_SYNC_COOLDOWN
};

volatile RxState rxState = IDLE;
volatile uint8_t startBitsSeen = 0;
volatile uint8_t bitIndex = 0;
volatile uint8_t shiftReg = 0;
volatile uint8_t decodedByte = 0;
volatile bool byteReady = false;
volatile bool frameError = false;
volatile uint32_t waitTicksRemaining = 0;

// Forward declarations for explicit IRQ attach/detach control
void onRxStartEdge();
inline void enableRxStartInterrupt();
inline void disableRxStartInterrupt();

// ===== Fast pin read for D4 our signal pin
// Read RX_PIN directly from VPORTC for low-latency sampling inside ISR paths.
inline uint8_t readRxFast()
{
   return bitRead(VPORTC.IN, 6);    // this is over 44x faster than digitalRead(RX_PIN) in the ISR context, and avoids 
                                    // the overhead of pinMode() checks.
}

// Toggle the scope/debug pin (D5/PB2) to visualize timing on external instruments.
inline void toggleScope()
{
    // PB2 = Arduino D5 (official Nano Every core mapping)
    bitToggle(VPORTB.OUT, 2); // toggle PB2 = Arduino D5
   // bitToggle(VPORTB.OUT, 2); // toggle PB2 = Arduino D5
}

// Stop TCB0 one-shot timing and mask its interrupt until the next frame arm.
inline void tcb0Stop()
{
    TCB0.CTRLA &= ~TCB_ENABLE_bm;  // halt TCB0 so no further one-shot counting occurs
    TCB0.INTCTRL &= ~TCB_CAPT_bm;  // mask CAPT interrupt while decoder is idle/resetting
}

// Arm TCB0 for a single timeout interval in timer ticks.
inline void tcb0ArmOneShot(uint16_t ccmp)
{
    TCB0.CNT = 0;                                // restart counting from zero for a fresh interval
    TCB0.CCMP = ccmp;                            // set one-shot timeout in timer ticks
    TCB0.INTFLAGS = TCB_CAPT_bm;                 // clear any pending CAPT flag (write-1-to-clear)
    TCB0.INTCTRL |= TCB_CAPT_bm;                 // unmask CAPT interrupt for this one-shot
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // apply prescaler and start TCB0
}

// Consume remaining wait time in UINT16_MAX-sized chunks and arm the next slice.
inline void tcb0ArmRemainingDelay()
{
    // TCB0.CCMP is 16-bit counter, so a single one-shot can wait at most UINT16_MAX ticks.
    // Long waits (like 80-bit preamble delay) are split into bounded chunks.
    uint16_t nextTicks = (waitTicksRemaining > UINT16_MAX)
        ? UINT16_MAX
        : static_cast<uint16_t>(waitTicksRemaining);

    // Consume this chunk and re-arm until waitTicksRemaining reaches zero.
    waitTicksRemaining -= nextTicks;
    tcb0ArmOneShot(nextTicks);
}

// Attach falling-edge interrupt on RX_PIN to detect the next frame start.
inline void enableRxStartInterrupt()
{
    attachInterrupt(digitalPinToInterrupt(RX_PIN), onRxStartEdge, FALLING);
}

// Detach RX_PIN edge interrupt while a frame is actively being decoded.
inline void disableRxStartInterrupt()
{
    detachInterrupt(digitalPinToInterrupt(RX_PIN));
}

// Rearm one-bit sample interval for start-bit or data-bit cadence.
inline void tcb0RearmBitSample()
{
    TCB0.CNT = 0;                 // restart one-shot timing from zero
    TCB0.CCMP = BIT_TICKS;        // next sample exactly one bit later
    TCB0.INTFLAGS = TCB_CAPT_bm;  // clear CAPT flag (write-1-to-clear)
}

// Enter frame-error cooldown and keep TCB0 running until the backoff delay expires.
inline void enterFrameErrorWait()
{
    // Cooldown prevents immediate re-sync on the same corrupted burst/noise event.
    // Waiting 100 bit-times gives the line time to return to idle framing before re-arming edge detect.
    frameError = true;
    rxState = FRAME_SYNC_COOLDOWN;
    waitTicksRemaining = ERROR_WAIT_TICKS;
    tcb0ArmRemainingDelay();
}

// Interrupt handler for falling edge on D4; this synchronizes us to an incoming frame.
// Called by GPIO interrupt on RX_PIN (D4). If decoder is IDLE, it disables edge 
// interrupts and arms the one-shot timer.
volatile int16_t edge_intrCounter = 0; // for debugging, counts how many times the GPIO edge ISR is entered
void onRxStartEdge()
{
    
    // Ignore retriggers while decoding a frame.
edge_intrCounter++; // increment edge ISR entry counter for debugging
    if (rxState != IDLE) {
        return;
    }

    // Confirm line still low on ISR entry.
    if (readRxFast() != LOW) {
        return;
    }
bitToggle(VPORTB.OUT, 2); // toggle PB2 = Arduino D5

    // One-shot arm: no more GPIO edges until this frame finishes.
    disableRxStartInterrupt();

    rxState = WAIT_FOR_FIRST_START_BIT;
    bitIndex = 0;
    shiftReg = 0;
    frameError = false;
    waitTicksRemaining = START_LOW_TICKS + HALF_BIT_TICKS; // wait for 80.5 bit-times before sampling first start bit
    tcb0ArmRemainingDelay(); // arm first delay slice toward the 80.5-bit first-sample point
bitToggle(VPORTB.OUT, 2); // toggle PB2 = Arduino D5
}


// Configure RX pin and TCB0 one-shot timer, then arm the frame-start edge interrupt.
void setupEventSystemForD4()
{
    pinMode(RX_PIN, INPUT_PULLUP);

    // TCB0 in single-shot mode, event capture/start enabled.
    TCB0.CTRLA = 0;                              // keep TCB0 disabled while configuring
    TCB0.CTRLB = TCB_CNTMODE_SINGLE_gc;          // one-shot mode (counter stops at compare)
TCB0.EVCTRL = 0;//TCB_CAPTEI_bm | TCB_EDGE_bm;   // allow event capture on selected edge
    TCB0.INTFLAGS = TCB_CAPT_bm;                 // clear stale CAPT flag before first arm
    TCB0.INTCTRL = 0;                            // leave CAPT interrupt masked until armed

    // Arm falling-edge start interrupt once at setup.
    enableRxStartInterrupt();
}

volatile int16_t tmr_isrCntr = 0; // for debugging, counts how many times the ISR is entered
volatile int16_t tmr_EarlyExit1 = 0; 
volatile int16_t tmr_EarlyExit2 = 0; 
volatile int16_t tmr_EarlyExit3 = 0; 
volatile int16_t tmr_EarlyExit4 = 0; 
volatile int16_t tmr_EarlyExit5 = 0; 
volatile int16_t tmr_EarlyExit6 = 0; 
volatile int16_t tmr_EarlyExit7 = 0; 
volatile int16_t tmr_EarlyExit8 = 0; 

// This is the vectored interrupt handler for TCB0 one-shot ISR drives the receive state machine after 
// a frame-start edge.  It chunks long preamble waits, validates start bits at bit-center sample points,
// shifts in 8 LSB-first data bits, and re-arms GPIO edge detection when done or on error.
ISR(TCB0_INT_vect)
{
tmr_isrCntr++;
    TCB0.INTFLAGS = TCB_CAPT_bm; // clear CAPT flag

    switch (rxState) {
        // Chunking is required because TCB0.CCMP is 16-bit (max 65535 ticks per one-shot).
        // Our initial wait (80-bit preamble + half-bit) is much longer than one timer period.
        // So we re-arm in slices, decrement waitTicksRemaining each ISR, and only continue when it reaches zero.
        // This avoids compare overflow/wrap and preserves accurate sample timing.
    case WAIT_FOR_FIRST_START_BIT:
        if (waitTicksRemaining > 0) {
            tcb0ArmRemainingDelay();
tmr_EarlyExit1++;
            return;
        }

        // At 80.5 bit-times we expect center of first of 5 start bits => HIGH.
        if (readRxFast() != HIGH) {
            enterFrameErrorWait();
tmr_EarlyExit2++;
            return;
        }

        // First start bit already validated at 80.5-bit sample.
        rxState = READING_START_BITS;
        startBitsSeen = 1;

        tcb0RearmBitSample(); // schedule next ISR at the center of the next start bit
tmr_EarlyExit3++;
        return;

    case READING_START_BITS:
        // Validate remaining start bits are HIGH.
        if (readRxFast() == LOW) {
            enterFrameErrorWait();
tmr_EarlyExit4++;
            return;
        }

        startBitsSeen++;
        if (startBitsSeen >= START_BITS) {
            // Start bits complete; next sample is first data bit.
            rxState = READING_DATA_BITS;
            bitIndex = 0;
            shiftReg = 0;
        }

        tcb0RearmBitSample(); // re-arm timer, keep one-bit cadence while validating start bits
        return;

    case READING_DATA_BITS: {
        uint8_t bit = readRxFast();
toggleScope();

//         // Store data bits LSB-first.
//         shiftReg |= (bit << bitIndex);
// tmr_EarlyExit5++;
//         bitIndex++;

         // Store data bits MSB-first (first sampled bit becomes bit 7).
        shiftReg = static_cast<uint8_t>((shiftReg << 1) | (bit & 0x01));
        bitIndex++;
        tmr_EarlyExit5++;

        if (bitIndex >= 8) {
            decodedByte = shiftReg;
            byteReady = true;
            rxState = IDLE;
            // Frame complete: stop bit-sampling timer and re-arm GPIO edge trigger for next frame.
            tcb0Stop();
            enableRxStartInterrupt();
toggleScope();
tmr_EarlyExit6++;
            return;
        }

        tcb0RearmBitSample(); // schedule next data-bit sample one bit-time later
toggleScope();
        return;
    }

    case FRAME_SYNC_COOLDOWN:
        if (waitTicksRemaining > 0) {
            tcb0ArmRemainingDelay();
            return;
        }
tmr_EarlyExit7++;
        // cooldown period -  Restart only after quiet time to avoid false retrigger loops on malformed frames.
        rxState = IDLE;
        tcb0Stop();
        enableRxStartInterrupt();
        return;

    case IDLE:
    default:
        tcb0Stop();
        enableRxStartInterrupt();
tmr_EarlyExit8++;
        return;
    }
}

int16_t auxBusReadDecodedByteImpl()
{
    int16_t result = -1;

    noInterrupts();
    if (byteReady) {
        result = static_cast<int16_t>(decodedByte);
        byteReady = false; // consume this byte
    }
    interrupts();

    return result;
}

void setup2Impl()
{
    //Serial.begin(115200);  // the serial port is initialized in main.cpp, so we don't need to do it here.
    Serial.print("\nBAUD: ");
    Serial.println(BAUD);
    Serial.print("BIT_TICKS: ");
    Serial.println(BIT_TICKS);
    Serial.print("HALF_BIT_TICKS: ");
    Serial.println(HALF_BIT_TICKS);
    Serial.print("START_LOW_TICKS: ");
    Serial.println(START_LOW_TICKS);


    pinMode(RX_PIN, INPUT_PULLUP);  // this is the KPA's AuxBus signal input, which is open-collector and requires a pull-up 
                                    //  to idle HIGH.
    pinMode(5, OUTPUT); // D5 = PB2 = scope output for ISR timing visualization
    bitClear(VPORTB.OUT, 2); //  PB2 = Arduino D5

    setupEventSystemForD4();
}

// this is used only for debugging the ISR and timing, not called from loop() in normal operation
void loop2Impl()
{
    if (byteReady) {
        // Snapshot shared data atomically so ISR updates cannot tear multi-step reads.
        noInterrupts();
        uint8_t b = decodedByte;
        byteReady = false;
        interrupts();

        Serial.print("Decoded byte: 0x");
        if (b < 16)
            Serial.print('0');
        Serial.println(b, HEX);
    }

    if (frameError) {
        // Clear frameError atomically with interrupts masked for ISR-safe handoff.
        noInterrupts();
        frameError = false;
        interrupts();
        Serial.println("Frame error");
    }

    static int32_t lastTime = 0;
    int16_t isrCount;
    int16_t e1;
    int16_t e2;
    int16_t e3;
    int16_t e4;
    int16_t e5;
    int16_t e6;
    int16_t e7;
    int16_t e8; 
    int16_t edgeCount;

    static uint32_t lastPrintMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= 15000UL) {
        // Take one coherent snapshot of all ISR-owned counters before printing.
        noInterrupts();
        isrCount = tmr_isrCntr;
        e1 = tmr_EarlyExit1;
        e2 = tmr_EarlyExit2;
        e3 = tmr_EarlyExit3;
        e4 = tmr_EarlyExit4;
        e5 = tmr_EarlyExit5;
        e6 = tmr_EarlyExit6;
        e7 = tmr_EarlyExit7;
        e8 = tmr_EarlyExit8;
        edgeCount = edge_intrCounter;
        interrupts();

        const bool isrChanged = (isrCount != lastTime);
        lastTime = isrCount;

        if (isrChanged) {
            lastPrintMs = nowMs;
            Serial.print("tmr ISR count: ");
            Serial.println(isrCount);
            Serial.print("tmr_EarlyExit1: ");
            Serial.println(e1);
            Serial.print("tmr_EarlyExit2: ");
            Serial.println(e2);
            Serial.print("tmr_EarlyExit3: ");
            Serial.println(e3);
            Serial.print("tmr_EarlyExit4: ");
            Serial.println(e4);
            Serial.print("tmr_EarlyExit5: ");
            Serial.println(e5);
            Serial.print("tmr_EarlyExit6: ");
            Serial.println(e6);
            Serial.print("tmr_EarlyExit7: ");
            Serial.println(e7);
            Serial.print("tmr_EarlyExit8: ");
            Serial.println(e8);
            Serial.print("edge_intrCounter: ");
            Serial.println(edgeCount);
        }
    }
}

} // namespace

int16_t auxBusReadDecodedByte()
{
    return auxBusReadDecodedByteImpl();
}

void setup2()
{
    setup2Impl();
}

void loop2()
{
    loop2Impl();
}
