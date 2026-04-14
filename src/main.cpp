#include <Arduino.h>

// ── Protocol structs ──────────────────────────────────────────────────────────
//
// FinderCommand (Pi → ESP32): 5 raw bytes, ~7 bytes once COBS-framed.
//   focus, zoom:  int8_t direction+magnitude. 0 = stop. ±1..±127 maps linearly
//                 to PWM 89..255 (35%..100% duty). Sign picks direction.
//   duration_ms:  0 = run until the next command; >0 = firmware auto-stops the
//                 affected motor(s) after N milliseconds (timed pulse).
//   crc:          CRC8 Dallas/Maxim (poly 0x31) over [focus, zoom, duration_ms].

struct FinderCommand {
    int8_t   focus;
    int8_t   zoom;
    uint16_t duration_ms;
    uint8_t  crc;
} __attribute__((packed));

struct FinderStatus {
    uint8_t  flags;            // bit0=focus_active, bit1=zoom_active
    uint8_t  endstops;         // bit0=f_min, bit1=f_max, bit2=z_min, bit3=z_max
    uint8_t  last_error;       // 0=none, 1=watchdog, 2=endstop_rejected, 3=bad_frame
    uint16_t focus_active_ms;  // accumulated drive time since last endstop reset
    uint16_t zoom_active_ms;
    uint8_t  focus_pwm;        // current duty cycle 0-255
    uint8_t  zoom_pwm;
    uint8_t  crc;
} __attribute__((packed));

// ── Pin definitions ───────────────────────────────────────────────────────────

#define FOCUS_IN1         5
#define FOCUS_IN2         4
#define ZOOM_IN3          7
#define ZOOM_IN4          6
#define FOCUS_ENDSTOP_MIN 2
#define FOCUS_ENDSTOP_MAX 3
#define ZOOM_ENDSTOP_MIN  1
#define ZOOM_ENDSTOP_MAX  0

// ── PWM ───────────────────────────────────────────────────────────────────────

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

// One LEDC channel per H-bridge input. Allocated in setup() via ledcSetup()
// and bound to the corresponding GPIO with ledcAttachPin(), then driven with
// ledcWrite(channel, duty) in the motor functions. arduino-esp32 v2.x API.
#define FOCUS_IN1_CH 0
#define FOCUS_IN2_CH 1
#define ZOOM_IN3_CH  2
#define ZOOM_IN4_CH  3

// ── Timing ───────────────────────────────────────────────────────────────────

#define WATCHDOG_TIMEOUT_MS 500
#define STATUS_INTERVAL_MS  100
#define DEBOUNCE_DELAY_MS   15

// ── Motor state ───────────────────────────────────────────────────────────────

bool    focusMotorActive    = false;
int     focusMotorDirection = 0;    // 0, 1, -1
uint8_t focusMotorPwm       = 0;

bool    zoomMotorActive     = false;
int     zoomMotorDirection  = 0;
uint8_t zoomMotorPwm        = 0;

// ── Active-time tracking (pseudo-encoder) ─────────────────────────────────────

uint32_t focusActiveAccumMs = 0;   // accumulated ms while motor was driven
uint32_t focusMoveStartMs   = 0;   // millis() when current run started
uint32_t zoomActiveAccumMs  = 0;
uint32_t zoomMoveStartMs    = 0;

// ── Timed pulse state ─────────────────────────────────────────────────────────

uint32_t focusPulseEndMs  = 0;
bool     focusPulseActive = false;
uint32_t zoomPulseEndMs   = 0;
bool     zoomPulseActive  = false;

// ── Endstop debounce ──────────────────────────────────────────────────────────

bool          focusMinLastState       = HIGH;
bool          focusMaxLastState       = HIGH;
bool          zoomMinLastState        = HIGH;
bool          zoomMaxLastState        = HIGH;
unsigned long focusMinLastTriggerTime = 0;
unsigned long focusMaxLastTriggerTime = 0;
unsigned long zoomMinLastTriggerTime  = 0;
unsigned long zoomMaxLastTriggerTime  = 0;

// ── Protocol state ────────────────────────────────────────────────────────────

bool     firstFrameReceived = false;
uint32_t lastValidFrameMs   = 0;
bool     watchdogTripped    = false;
uint8_t  lastError          = 0;

uint8_t  rxBuf[32];
uint8_t  rxLen = 0;

uint32_t lastStatusMs = 0;

// ── CRC8 Dallas/Maxim (polynomial 0x31) ──────────────────────────────────────

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

// ── COBS encode ───────────────────────────────────────────────────────────────
// out must be at least (len + 2) bytes. Returns total bytes written (incl. 0x00).

static size_t cobsEncode(const uint8_t* in, size_t len, uint8_t* out) {
    size_t  outIdx  = 0;
    size_t  codeIdx = outIdx++;
    uint8_t code    = 1;

    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0x00) {
            out[codeIdx] = code;
            codeIdx = outIdx++;
            code = 1;
        } else {
            out[outIdx++] = in[i];
            code++;
            if (code == 0xFF) {
                out[codeIdx] = code;
                codeIdx = outIdx++;
                code = 1;
            }
        }
    }
    out[codeIdx]  = code;
    out[outIdx++] = 0x00;
    return outIdx;
}

// ── COBS decode ───────────────────────────────────────────────────────────────
// in: encoded bytes WITHOUT the 0x00 terminator. Returns decoded length, 0 on error.

static size_t cobsDecode(const uint8_t* in, size_t len, uint8_t* out) {
    size_t inIdx  = 0;
    size_t outIdx = 0;

    while (inIdx < len) {
        uint8_t code = in[inIdx++];
        if (code == 0) return 0;

        for (uint8_t i = 1; i < code; i++) {
            if (inIdx >= len) return 0;
            out[outIdx++] = in[inIdx++];
        }
        if (code != 0xFF && inIdx < len)
            out[outIdx++] = 0x00;
    }
    return outIdx;
}

// ── Motor stop functions ──────────────────────────────────────────────────────

void focusStop() {
    if (focusMotorActive)
        focusActiveAccumMs += millis() - focusMoveStartMs;

    // MX1508 brake mode: both inputs HIGH. At 8-bit resolution the max duty
    // (255) is ~99.6% HIGH — the 0.2 µs LOW sliver per 50 µs period is below
    // the bridge's input response, so this is effectively a continuous brake.
    ledcWrite(FOCUS_IN1_CH, 255);
    ledcWrite(FOCUS_IN2_CH, 255);

    focusMotorActive    = false;
    focusMotorDirection = 0;
    focusMotorPwm       = 0;
    focusPulseActive    = false;
}

void zoomStop() {
    if (zoomMotorActive)
        zoomActiveAccumMs += millis() - zoomMoveStartMs;

    ledcWrite(ZOOM_IN3_CH, 255);
    ledcWrite(ZOOM_IN4_CH, 255);

    zoomMotorActive    = false;
    zoomMotorDirection = 0;
    zoomMotorPwm       = 0;
    zoomPulseActive    = false;
}

// ── Motor move functions ──────────────────────────────────────────────────────

void focusForward(uint8_t pwm) {
    if (digitalRead(FOCUS_ENDSTOP_MAX) == LOW) {
        focusStop();
        lastError = 2;
        return;
    }
    // Route direction reversals through brake to avoid hammering the H-bridge
    // with an undefined transient while one PWM pin tears down and the other
    // spins up. Same-direction PWM changes skip the stop and just retune duty.
    if (focusMotorActive && focusMotorDirection != 1)
        focusStop();

    ledcWrite(FOCUS_IN2_CH, 0);
    ledcWrite(FOCUS_IN1_CH, pwm);

    if (!focusMotorActive) focusMoveStartMs = millis();
    focusMotorActive    = true;
    focusMotorDirection = 1;
    focusMotorPwm       = pwm;
}

void focusBack(uint8_t pwm) {
    if (digitalRead(FOCUS_ENDSTOP_MIN) == LOW) {
        focusStop();
        lastError = 2;
        return;
    }
    if (focusMotorActive && focusMotorDirection != -1)
        focusStop();

    ledcWrite(FOCUS_IN1_CH, 0);
    ledcWrite(FOCUS_IN2_CH, pwm);

    if (!focusMotorActive) focusMoveStartMs = millis();
    focusMotorActive    = true;
    focusMotorDirection = -1;
    focusMotorPwm       = pwm;
}

void zoomForward(uint8_t pwm) {
    if (digitalRead(ZOOM_ENDSTOP_MAX) == LOW) {
        zoomStop();
        lastError = 2;
        return;
    }
    if (zoomMotorActive && zoomMotorDirection != 1)
        zoomStop();

    ledcWrite(ZOOM_IN4_CH, 0);
    ledcWrite(ZOOM_IN3_CH, pwm);

    if (!zoomMotorActive) zoomMoveStartMs = millis();
    zoomMotorActive    = true;
    zoomMotorDirection = 1;
    zoomMotorPwm       = pwm;
}

void zoomBack(uint8_t pwm) {
    if (digitalRead(ZOOM_ENDSTOP_MIN) == LOW) {
        zoomStop();
        lastError = 2;
        return;
    }
    if (zoomMotorActive && zoomMotorDirection != -1)
        zoomStop();

    ledcWrite(ZOOM_IN3_CH, 0);
    ledcWrite(ZOOM_IN4_CH, pwm);

    if (!zoomMotorActive) zoomMoveStartMs = millis();
    zoomMotorActive    = true;
    zoomMotorDirection = -1;
    zoomMotorPwm       = pwm;
}

// ── Command dispatch ──────────────────────────────────────────────────────────

static void processMotorCommand(const FinderCommand& cmd) {
    if (cmd.focus > 0)
        focusForward((uint8_t)map(abs(cmd.focus), 1, 127, 89, 255));
    else if (cmd.focus < 0)
        focusBack((uint8_t)map(abs(cmd.focus), 1, 127, 89, 255));
    else
        focusStop();

    if (cmd.focus != 0 && cmd.duration_ms > 0) {
        focusPulseEndMs  = millis() + cmd.duration_ms;
        focusPulseActive = true;
    } else {
        focusPulseActive = false;
    }

    if (cmd.zoom > 0)
        zoomForward((uint8_t)map(abs(cmd.zoom), 1, 127, 89, 255));
    else if (cmd.zoom < 0)
        zoomBack((uint8_t)map(abs(cmd.zoom), 1, 127, 89, 255));
    else
        zoomStop();

    if (cmd.zoom != 0 && cmd.duration_ms > 0) {
        zoomPulseEndMs  = millis() + cmd.duration_ms;
        zoomPulseActive = true;
    } else {
        zoomPulseActive = false;
    }
}

// ── Status frame transmission ─────────────────────────────────────────────────

static void sendStatus() {
    uint32_t now = millis();

    // Include current run time without modifying the accumulators
    uint32_t fTotal = focusActiveAccumMs + (focusMotorActive ? (now - focusMoveStartMs) : 0);
    uint32_t zTotal = zoomActiveAccumMs  + (zoomMotorActive  ? (now - zoomMoveStartMs)  : 0);

    bool fMin = (digitalRead(FOCUS_ENDSTOP_MIN) == LOW);
    bool fMax = (digitalRead(FOCUS_ENDSTOP_MAX) == LOW);
    bool zMin = (digitalRead(ZOOM_ENDSTOP_MIN)  == LOW);
    bool zMax = (digitalRead(ZOOM_ENDSTOP_MAX)  == LOW);

    FinderStatus st;
    st.flags           = (focusMotorActive ? 0x01 : 0x00) | (zoomMotorActive ? 0x02 : 0x00);
    st.endstops        = (fMin ? 0x01 : 0) | (fMax ? 0x02 : 0) | (zMin ? 0x04 : 0) | (zMax ? 0x08 : 0);
    st.last_error      = lastError;
    st.focus_active_ms = (uint16_t)(fTotal > 65535UL ? 65535UL : fTotal);
    st.zoom_active_ms  = (uint16_t)(zTotal > 65535UL ? 65535UL : zTotal);
    st.focus_pwm       = focusMotorPwm;
    st.zoom_pwm        = zoomMotorPwm;
    st.crc             = crc8((const uint8_t*)&st, sizeof(st) - 1);

    uint8_t frame[sizeof(FinderStatus) + 2];
    size_t  frameLen = cobsEncode((const uint8_t*)&st, sizeof(st), frame);
    Serial.write(frame, frameLen);
}

// ── setup() ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);
    while (!Serial && millis() < 5000) delay(100);

    pinMode(FOCUS_ENDSTOP_MIN, INPUT_PULLUP);
    pinMode(FOCUS_ENDSTOP_MAX, INPUT_PULLUP);
    pinMode(ZOOM_ENDSTOP_MIN,  INPUT_PULLUP);
    pinMode(ZOOM_ENDSTOP_MAX,  INPUT_PULLUP);

    // Allocate a dedicated LEDC channel per H-bridge input and bind the pin
    // once, here in setup(). Motor functions then only call ledcWrite(ch, duty)
    // — no per-call pin-mode / detach / attach dance.
    ledcSetup(FOCUS_IN1_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FOCUS_IN1, FOCUS_IN1_CH);
    ledcSetup(FOCUS_IN2_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FOCUS_IN2, FOCUS_IN2_CH);
    ledcSetup(ZOOM_IN3_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ZOOM_IN3, ZOOM_IN3_CH);
    ledcSetup(ZOOM_IN4_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ZOOM_IN4, ZOOM_IN4_CH);

    focusStop();
    zoomStop();

    focusMinLastState = digitalRead(FOCUS_ENDSTOP_MIN);
    focusMaxLastState = digitalRead(FOCUS_ENDSTOP_MAX);
    zoomMinLastState  = digitalRead(ZOOM_ENDSTOP_MIN);
    zoomMaxLastState  = digitalRead(ZOOM_ENDSTOP_MAX);

    // Arm the software watchdog immediately: a stuck motor from a previous
    // session (or any pre-comms fault) is now guaranteed to be force-stopped
    // within WATCHDOG_TIMEOUT_MS even if the host never connects. The first
    // valid command from the relay clears lastError back to 0.
    lastValidFrameMs   = millis();
    firstFrameReceived = true;

    lastStatusMs = millis();
}

// ── loop() ───────────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();

    // ── Inbound COBS frame accumulation ──────────────────────────────────────

    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (b == 0x00) {
            // Frame delimiter — attempt decode
            if (rxLen > 0) {
                uint8_t decoded[32];
                size_t  dLen = cobsDecode(rxBuf, rxLen, decoded);

                if (dLen == sizeof(FinderCommand)) {
                    FinderCommand* cmd      = (FinderCommand*)decoded;
                    uint8_t        expected = crc8(decoded, sizeof(FinderCommand) - 1);
                    if (cmd->crc == expected) {
                        firstFrameReceived = true;
                        lastValidFrameMs   = now;
                        watchdogTripped    = false;
                        lastError          = 0;
                        processMotorCommand(*cmd);
                    } else {
                        lastError = 3;  // bad_frame
                    }
                } else {
                    lastError = 3;
                }
                rxLen = 0;
            }
        } else {
            if (rxLen < sizeof(rxBuf)) {
                rxBuf[rxLen++] = b;
            } else {
                lastError = 3;  // bad_frame — overflow, resync
                rxLen      = 0;
            }
        }
    }

    // ── Software watchdog ─────────────────────────────────────────────────────

    if (firstFrameReceived && !watchdogTripped && (now - lastValidFrameMs > WATCHDOG_TIMEOUT_MS)) {
        focusStop();
        zoomStop();
        lastError      = 1;
        watchdogTripped = true;
    }

    // ── Timed pulse expiry ────────────────────────────────────────────────────

    if (focusPulseActive && now >= focusPulseEndMs)
        focusStop();

    if (zoomPulseActive && now >= zoomPulseEndMs)
        zoomStop();

    // ── Endstop polling with debounce ─────────────────────────────────────────

    bool focusMinState = digitalRead(FOCUS_ENDSTOP_MIN);
    if (focusMinState != focusMinLastState && (now - focusMinLastTriggerTime) > DEBOUNCE_DELAY_MS) {
        if (focusMinState == LOW && focusMotorActive && focusMotorDirection == -1) {
            // Endstop = known home. Clear the active flag first so focusStop()
            // skips its accumulator path, then zero the pseudo-encoder directly.
            focusMotorActive   = false;
            focusStop();
            focusActiveAccumMs = 0;
        }
        focusMinLastState       = focusMinState;
        focusMinLastTriggerTime = now;
    }

    bool focusMaxState = digitalRead(FOCUS_ENDSTOP_MAX);
    if (focusMaxState != focusMaxLastState && (now - focusMaxLastTriggerTime) > DEBOUNCE_DELAY_MS) {
        if (focusMaxState == LOW && focusMotorActive && focusMotorDirection == 1) {
            focusStop();
        }
        focusMaxLastState       = focusMaxState;
        focusMaxLastTriggerTime = now;
    }

    bool zoomMinState = digitalRead(ZOOM_ENDSTOP_MIN);
    if (zoomMinState != zoomMinLastState && (now - zoomMinLastTriggerTime) > DEBOUNCE_DELAY_MS) {
        if (zoomMinState == LOW && zoomMotorActive && zoomMotorDirection == -1) {
            zoomMotorActive   = false;
            zoomStop();
            zoomActiveAccumMs = 0;
        }
        zoomMinLastState       = zoomMinState;
        zoomMinLastTriggerTime = now;
    }

    bool zoomMaxState = digitalRead(ZOOM_ENDSTOP_MAX);
    if (zoomMaxState != zoomMaxLastState && (now - zoomMaxLastTriggerTime) > DEBOUNCE_DELAY_MS) {
        if (zoomMaxState == LOW && zoomMotorActive && zoomMotorDirection == 1) {
            zoomStop();
        }
        zoomMaxLastState       = zoomMaxState;
        zoomMaxLastTriggerTime = now;
    }

    // ── Status broadcast at 10 Hz ─────────────────────────────────────────────

    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        sendStatus();
        lastStatusMs = now;
    }

    delay(1);
    yield();
}
