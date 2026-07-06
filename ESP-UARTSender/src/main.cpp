#include <Arduino.h>

#define RXD1_PIN 16
#define TXD1_PIN 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define MSG_TYPE_BUTTON 0x01
#define NUM_BUTTONS 2
#define BUTTON_DEBOUNCE_MS 20
#define KEYPAD_ROWS 3
#define KEYPAD_COLS 4
#define KEYPAD_BUTTON_COUNT (KEYPAD_ROWS * KEYPAD_COLS)
#define KEYPAD_DEBOUNCE_MS 20
#define KEYPAD_BUTTON_OFFSET NUM_BUTTONS
#define ENCODER_PULSE_DURATION_MS 100
#define ENCODER_DEBOUNCE_MS 150
#define NUM_ENCODERS 3
#define NUM_VIRTUAL_BUTTONS 12
#define VIRTUAL_BUTTON_OFFSET (KEYPAD_BUTTON_OFFSET + KEYPAD_BUTTON_COUNT)

uint8_t button_pins[NUM_BUTTONS] = { 13, 32 };
bool button_state[NUM_BUTTONS] = { false, false };
bool last_button_state[NUM_BUTTONS] = { false, false };
unsigned long last_debounce_time[NUM_BUTTONS] = { 0, 0 };
const uint8_t keypad_row_pins[KEYPAD_ROWS] = { 14, 27, 26 }; 
const uint8_t keypad_col_pins[KEYPAD_COLS] = { 25, 33, 18, 19 };
bool keypad_debounced_state[KEYPAD_BUTTON_COUNT] = { false };
bool keypad_last_raw_state[KEYPAD_BUTTON_COUNT] = { false };
uint32_t keypad_last_change_ms[KEYPAD_BUTTON_COUNT] = { 0 };
unsigned long virtual_btn_timer[NUM_VIRTUAL_BUTTONS] = { 0 };
bool virtual_btn_state[NUM_VIRTUAL_BUTTONS] = { false };

struct EncoderData {
    uint8_t pinCLK;
    uint8_t pinDT;
    uint8_t pinSW;
    int lastCLK;
    bool altMode;
    uint8_t idCW_Norm;
    uint8_t idCCW_Norm;
    uint8_t idCW_Alt;
    uint8_t idCCW_Alt;
    unsigned long debounceSW;
    bool lastSW;
    unsigned long lastRotationTime; 
    bool lastDirectionCW;
};

EncoderData encoders[NUM_ENCODERS] = {
    {23, 22, 1, 0, false, 14, 15, 16, 17, 0, HIGH},
    {4, 2, 15, 0, false, 18, 19, 20, 21, 0, HIGH},
    {5, 21, 3, 0, false, 22, 23, 24, 25, 0, HIGH}
};

uint8_t calculate_CRC(uint8_t *data) {
    uint8_t crc = 0;
    for (int i = 0; i < 5; i++) {
        crc ^= data[i];
    }
    return crc;
}

void send_button_event(uint8_t id, bool state) {
    uint8_t packet[6];
    packet[0] = START_BYTE;
    packet[1] = MSG_TYPE_BUTTON;
    packet[2] = id;
    packet[3] = state ? 1 : 0;
    packet[4] = 0;
    packet[5] = calculate_CRC(packet);
    Serial1.write(packet, 6);
}

void trigger_virtual_button(uint8_t id) {
    uint8_t idx = id - VIRTUAL_BUTTON_OFFSET;
    if (idx < NUM_VIRTUAL_BUTTONS) {
        if (!virtual_btn_state[idx]) {
            virtual_btn_state[idx] = true;
            send_button_event(id, true);
        }
        virtual_btn_timer[idx] = millis();
    }
}

void update_encoders() {
    unsigned long now = millis();
    for (int i = 0; i < NUM_ENCODERS; i++) {
        bool sw_state = digitalRead(encoders[i].pinSW);
        if (sw_state == LOW && encoders[i].lastSW == HIGH && (now - encoders[i].debounceSW > ENCODER_DEBOUNCE_MS)) {
            encoders[i].altMode = !encoders[i].altMode;
            encoders[i].debounceSW = now;
        }
        encoders[i].lastSW = sw_state;
        int clk_state = digitalRead(encoders[i].pinCLK);
        if (clk_state != encoders[i].lastCLK && clk_state == LOW) {
            int dt_state = digitalRead(encoders[i].pinDT);
            bool isCW = (dt_state == HIGH);
            bool isReversal = (isCW != encoders[i].lastDirectionCW);
            if (isReversal && (now - encoders[i].lastRotationTime < ENCODER_DEBOUNCE_MS)) {
            } else {
                encoders[i].lastDirectionCW = isCW;
                encoders[i].lastRotationTime = now;
                if (isCW) {
                    trigger_virtual_button(encoders[i].altMode ? encoders[i].idCW_Alt : encoders[i].idCW_Norm);
                } else {
                    trigger_virtual_button(encoders[i].altMode ? encoders[i].idCCW_Alt : encoders[i].idCCW_Norm);
                }
            }
        }
        encoders[i].lastCLK = clk_state;
    }
    for (uint8_t i = 0; i < NUM_VIRTUAL_BUTTONS; i++) {
        if (virtual_btn_state[i]) {
            if (now - virtual_btn_timer[i] >= ENCODER_PULSE_DURATION_MS) {
                virtual_btn_state[i] = false;
                send_button_event(VIRTUAL_BUTTON_OFFSET + i, false);
            }
        }
    }
}

void keypad_init() {
    for (uint8_t i = 0; i < KEYPAD_ROWS; i++) {
        pinMode(keypad_row_pins[i], OUTPUT);
        digitalWrite(keypad_row_pins[i], HIGH);
    }
    for (uint8_t i = 0; i < KEYPAD_COLS; i++) {
        pinMode(keypad_col_pins[i], INPUT_PULLUP);
    }
}

uint32_t scan_keypad_raw_mask() {
    uint32_t mask = 0;
    for (uint8_t row = 0; row < KEYPAD_ROWS; row++) {
        digitalWrite(keypad_row_pins[row], LOW);
        delayMicroseconds(3);
        for (uint8_t col = 0; col < KEYPAD_COLS; col++) {
            if (digitalRead(keypad_col_pins[col]) == LOW) {
                mask |= (1u << ((row * KEYPAD_COLS) + col));
            }
        }
        digitalWrite(keypad_row_pins[row], HIGH);
    }
    return mask;
}

void update_keypad_buttons() {
    const uint32_t now_ms = millis();
    const uint32_t raw_mask = scan_keypad_raw_mask();
    for (uint8_t i = 0; i < KEYPAD_BUTTON_COUNT; i++) {
        const bool raw_pressed = (raw_mask & (1u << i)) != 0;
        if (raw_pressed != keypad_last_raw_state[i]) {
            keypad_last_raw_state[i] = raw_pressed;
            keypad_last_change_ms[i] = now_ms;
        }
        if ((uint32_t)(now_ms - keypad_last_change_ms[i]) >= KEYPAD_DEBOUNCE_MS) {
            if (keypad_debounced_state[i] != raw_pressed) {
                keypad_debounced_state[i] = raw_pressed;
                send_button_event(KEYPAD_BUTTON_OFFSET + i, keypad_debounced_state[i]);
            }
        }
    }
}

void setup() {
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(button_pins[i], INPUT_PULLUP);
    }
    keypad_init();
    for (int i = 0; i < NUM_ENCODERS; i++) {
        pinMode(encoders[i].pinCLK, INPUT_PULLUP);
        pinMode(encoders[i].pinDT, INPUT_PULLUP);
        pinMode(encoders[i].pinSW, INPUT_PULLUP);
        encoders[i].lastCLK = digitalRead(encoders[i].pinCLK);
    }
}

void loop() {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        bool reading = digitalRead(button_pins[i]);
        if (reading != last_button_state[i]) {
            last_debounce_time[i] = millis();
        }
        if ((millis() - last_debounce_time[i]) > BUTTON_DEBOUNCE_MS) {
            if (reading != button_state[i]) {
                button_state[i] = reading;
                send_button_event(i, button_state[i]);
            }
        }
        last_button_state[i] = reading;
    }
    update_keypad_buttons();
    update_encoders();
}
