#include <Arduino.h>
#include <Preferences.h>
#include "driver/pcnt.h"

#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define PCNT_UNIT_USED PCNT_UNIT_0

#define HALL_ACCEL_PIN 14
#define HALL_BRAKE_PIN 13
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES_MS 10

#define PEDAL_FILTER_SIZE 16

#define RXD1 18
#define TXD1 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define PACKET_SIZE 6
#define MSG_TYPE_BUTTON 0x01
#define MSG_TYPE_ENCODER 0x02
#define MSG_TYPE_ANALOG 0x03

#define KEYPAD_ROW_0 1
#define KEYPAD_ROW_1 2
#define KEYPAD_ROW_2 47
#define KEYPAD_ROW_3 21
#define KEYPAD_COL_0 11
#define KEYPAD_COL_1 10
#define KEYPAD_COL_2 9
#define KEYPAD_COL_3 8

#define BRIDGE_START_BYTE 0xAB
#define BRIDGE_PKT_TELEMETRY 0x10
#define BRIDGE_PKT_TORQUE_CMD 0x20
#define BRIDGE_PKT_CONTROL_CMD 0x30
#define BRIDGE_TELEMETRY_INTERVAL_MS 5

#define BRIDGE_CMD_STARTUP 0x01
#define STARTUP_FLAG_AUTO_CALIB 0x0001
#define STARTUP_FLAG_SKIP_SELF_TEST 0x0002

#define BTS_RPWM_PIN 6
#define BTS_LPWM_PIN 7
#define BTS_REN_PIN 15
#define BTS_LEN_PIN 16
#define BTS_PWM_CH_R 0
#define BTS_PWM_CH_L 1
#define BTS_PWM_FREQ 20000
#define BTS_PWM_RES_BITS 10
#define BTS_PWM_MAX ((1 << BTS_PWM_RES_BITS) - 1)

#define FFB_ABS_MAX_VAL 1000
#define FFB_DEADZONE 5

#define FFB_POS_KP 0.28f
#define FFB_POS_KI 0.6f
#define FFB_POS_KD 0.0f
#define FFB_POS_I_LIMIT 270.0f

#define MOTOR_SELF_TEST_TORQUE_LIMIT 300
#define MOTOR_SELF_TEST_POS_TOL 60
#define MOTOR_SELF_TEST_HOLD_TIME_MS 500
#define MOTOR_SELF_TEST_TIMEOUT_MS 5000
#define MOTOR_SELF_TEST_POSITION 2250 

#define KEYPAD_BUTTON_COUNT 16
#define SECONDARY_BUTTON_OFFSET 0
#define SECONDARY_BUTTON_COUNT 26
#define KEYPAD_DEBOUNCE_MS 20

int16_t joyX = 0;
int16_t joyY = 0;
int16_t joyZ = 0;
uint64_t keypad_buttons = 0;
uint64_t secondary_buttons = 0;
uint64_t hid_buttons = 0;

uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;

const uint8_t keypad_row_pins[4] = {KEYPAD_ROW_0, KEYPAD_ROW_1, KEYPAD_ROW_2, KEYPAD_ROW_3};
const uint8_t keypad_col_pins[4] = {KEYPAD_COL_0, KEYPAD_COL_1, KEYPAD_COL_2, KEYPAD_COL_3};
bool keypad_debounced_state[KEYPAD_BUTTON_COUNT] = {false};
bool keypad_last_debounced_state[KEYPAD_BUTTON_COUNT] = {false};
bool keypad_last_raw_state[KEYPAD_BUTTON_COUNT] = {false};
uint32_t keypad_last_change_ms[KEYPAD_BUTTON_COUNT] = {0};

uint16_t hall_accel_val = 0, hall_accel_min = 0, hall_accel_max = 4095;
uint16_t hall_brake_val = 0, hall_brake_min = 0, hall_brake_max = 4095;

uint8_t bridge_rx_buf[32];
uint8_t bridge_rx_idx = 0;
uint8_t bridge_rx_expected = 0;

volatile int16_t ffb_torque_value = 0;
volatile uint8_t ffb_device_gain = 255;
bool ffb_software_enabled = true;

int16_t ffb_position = 0;
int32_t ffb_position_offset = 0;
int32_t ffb_last_position = 0;
int32_t ffb_last_position_error = 0;
int32_t ffb_position_integral = 0;
float ffb_position_derivative = 0.0f;
uint32_t ffb_position_time_us = 0;
uint32_t ffb_last_position_time_us = 0;

uint32_t last_bridge_telem_ms = 0;

bool bridge_startup_received = false;
uint16_t bridge_startup_flags = 0;

const int16_t wheel_ranges[4] = {2250, 4500, 6750, 9000};
int8_t current_range_idx = 2;

uint16_t accel_buffer[PEDAL_FILTER_SIZE] = {0};
uint16_t brake_buffer[PEDAL_FILTER_SIZE] = {0};
uint8_t pedal_filter_idx = 0;
uint32_t accel_sum = 0;
uint32_t brake_sum = 0;
bool pedals_filter_initialized = false;

void init_pedal_filter() {
    uint16_t init_accel = analogRead(HALL_ACCEL_PIN);
    uint16_t init_brake = analogRead(HALL_BRAKE_PIN);
    for (int i = 0; i < PEDAL_FILTER_SIZE; i++) {
        accel_buffer[i] = init_accel;
        brake_buffer[i] = init_brake;
    }
    accel_sum = init_accel * PEDAL_FILTER_SIZE;
    brake_sum = init_brake * PEDAL_FILTER_SIZE;
    pedals_filter_initialized = true;
}

void update_pedal_filter() {
    if (!pedals_filter_initialized) {
        init_pedal_filter();
    }
    uint16_t raw_accel = analogRead(HALL_ACCEL_PIN);
    uint16_t raw_brake = analogRead(HALL_BRAKE_PIN);
    accel_sum -= accel_buffer[pedal_filter_idx];
    accel_buffer[pedal_filter_idx] = raw_accel;
    accel_sum += raw_accel;
    brake_sum -= brake_buffer[pedal_filter_idx];
    brake_buffer[pedal_filter_idx] = raw_brake;
    brake_sum += raw_brake;
    pedal_filter_idx = (pedal_filter_idx + 1) % PEDAL_FILTER_SIZE;
}

uint16_t get_filtered_accel_raw() {
    return accel_sum / PEDAL_FILTER_SIZE;
}

uint16_t get_filtered_brake_raw() {
    return brake_sum / PEDAL_FILTER_SIZE;
}

void encoder_init() {
    pcnt_config_t pcnt_config_ch0 = {};
    pcnt_config_ch0.pulse_gpio_num = ENCODER_PIN_A;
    pcnt_config_ch0.ctrl_gpio_num = ENCODER_PIN_B;
    pcnt_config_ch0.channel = PCNT_CHANNEL_0;
    pcnt_config_ch0.unit = PCNT_UNIT_USED;
    pcnt_config_ch0.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_ch0.neg_mode = PCNT_COUNT_INC;
    pcnt_config_ch0.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_ch0.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_ch0.counter_h_lim = 32767;
    pcnt_config_ch0.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_ch0);
    pcnt_config_t pcnt_config_ch1 = {};
    pcnt_config_ch1.pulse_gpio_num = ENCODER_PIN_B;
    pcnt_config_ch1.ctrl_gpio_num = ENCODER_PIN_A;
    pcnt_config_ch1.channel = PCNT_CHANNEL_1;
    pcnt_config_ch1.unit = PCNT_UNIT_USED;
    pcnt_config_ch1.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_ch1.neg_mode = PCNT_COUNT_INC;
    pcnt_config_ch1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_ch1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_ch1.counter_h_lim = 32767;
    pcnt_config_ch1.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_ch1);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_counter_resume(PCNT_UNIT_USED);
}

void hall_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(HALL_ACCEL_PIN, ADC_11db);
    analogSetPinAttenuation(HALL_BRAKE_PIN, ADC_11db);
    pinMode(HALL_ACCEL_PIN, INPUT);
    pinMode(HALL_BRAKE_PIN, INPUT);
}

bool hall_calib_values_valid(uint16_t accel_min, uint16_t accel_max, uint16_t brake_min, uint16_t brake_max) {
    if (accel_min == accel_max || brake_min == brake_max) {
        return false;
    }
    if (accel_min > 4095 || accel_max > 4095 || brake_min > 4095 || brake_max > 4095) {
        return false;
    }
    return true;
}

bool load_hall_calib_from_flash() {
    Preferences prefs;
    if (!prefs.begin("pedal_calib", true)) {
        return false;
    }
    if (!prefs.getBool("valid", false)) {
        prefs.end(); return false;
    }
    hall_accel_min = prefs.getUShort("acc_min", 0);
    hall_accel_max = prefs.getUShort("acc_max", 4095);
    hall_brake_min = prefs.getUShort("brk_min", 0);
    hall_brake_max = prefs.getUShort("brk_max", 4095);
    prefs.end();
    return hall_calib_values_valid(hall_accel_min, hall_accel_max, hall_brake_min, hall_brake_max);
}

bool save_hall_calib_to_flash() {
    if (!hall_calib_values_valid(hall_accel_min, hall_accel_max, hall_brake_min, hall_brake_max)) {
        return false;
    }
    Preferences prefs;
    if (!prefs.begin("pedal_calib", false)) {
        return false;
    }
    prefs.putUShort("acc_min", hall_accel_min);
    prefs.putUShort("acc_max", hall_accel_max);
    prefs.putUShort("brk_min", hall_brake_min);
    prefs.putUShort("brk_max", hall_brake_max);
    prefs.putBool("valid", true);
    prefs.end();
    return true;
}

void keypad_init() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(keypad_row_pins[i], OUTPUT);
        digitalWrite(keypad_row_pins[i], HIGH);
        pinMode(keypad_col_pins[i], INPUT_PULLUP);
    }
}

uint32_t scan_keypad_raw_mask() {
    uint32_t mask = 0;
    for (uint8_t row = 0; row < 4; row++) {
        digitalWrite(keypad_row_pins[row], LOW);
        delayMicroseconds(3);
        for (uint8_t col = 0; col < 4; col++) {
            if (digitalRead(keypad_col_pins[col]) == LOW) {
                mask |= (1u << (row * 4 + col));
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
            keypad_debounced_state[i] = raw_pressed;
        }
    }
    bool btn3_pressed = keypad_debounced_state[3] && !keypad_last_debounced_state[3];
    bool btn7_pressed = keypad_debounced_state[7] && !keypad_last_debounced_state[7];
    bool btn11_pressed = keypad_debounced_state[11] && !keypad_last_debounced_state[11];
    bool btn11_released = !keypad_debounced_state[11] && keypad_last_debounced_state[11];
    bool btn12_pressed = keypad_debounced_state[12] && !keypad_last_debounced_state[12];
    bool btn15_pressed = keypad_debounced_state[15] && !keypad_last_debounced_state[15];
    if (btn3_pressed && current_range_idx > 0) {
        current_range_idx--;
    }
    if (btn7_pressed && current_range_idx < 3) {
        current_range_idx++;
    }
    if (btn12_pressed) {
        int16_t raw_pos;
        pcnt_get_counter_value(PCNT_UNIT_USED, &raw_pos);
        ffb_position_offset = raw_pos;
    }
    if (btn11_pressed) {
        uint16_t temp_accel_min = 0;
        uint16_t temp_brake_min = 0;
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            uint16_t acc_val = analogRead(HALL_ACCEL_PIN);
            uint16_t brk_val = analogRead(HALL_BRAKE_PIN);
            if (acc_val > temp_accel_min) {
                temp_accel_min = acc_val;
            }
            if (brk_val > temp_brake_min) {
                temp_brake_min = brk_val;
            }
            delay(CALIB_DELAY_BETWEEN_SAMPLES_MS);
        }
        hall_accel_min = temp_accel_min;
        hall_brake_min = temp_brake_min;
    }
    if (btn11_released) {
        uint16_t temp_accel_max = 4095;
        uint16_t temp_brake_max = 4095;
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            uint16_t acc_val = analogRead(HALL_ACCEL_PIN);
            uint16_t brk_val = analogRead(HALL_BRAKE_PIN);
            if (acc_val < temp_accel_max) {
                temp_accel_max = acc_val;
            }
            if (brk_val < temp_brake_max) {
                temp_brake_max = brk_val;
            }
            delay(CALIB_DELAY_BETWEEN_SAMPLES_MS);
        }
        hall_accel_max = temp_accel_max;
        hall_brake_max = temp_brake_max;
        save_hall_calib_to_flash();
    }
    if (btn15_pressed) {
        ffb_software_enabled = !ffb_software_enabled;
    }
    uint64_t new_keypad_bits = 0;
    if (keypad_debounced_state[0]) {
        new_keypad_bits |= (1ULL << 26);
    }
    if (keypad_debounced_state[2]) {
        new_keypad_bits |= (1ULL << 27);
    }
    if (keypad_debounced_state[5]) {
        new_keypad_bits |= (1ULL << 28);
    }
    if (keypad_debounced_state[8]) {
        new_keypad_bits |= (1ULL << 29);
    }
    if (keypad_debounced_state[10]) {
        new_keypad_bits |= (1ULL << 30);
    }
    if (keypad_debounced_state[13]) {
        new_keypad_bits |= (1ULL << 31);
    }
    if (keypad_debounced_state[1]) {
        new_keypad_bits |= (1ULL << 32);
    }
    if (keypad_debounced_state[4]) {
        new_keypad_bits |= (1ULL << 33);
    }
    if (keypad_debounced_state[6]) {
        new_keypad_bits |= (1ULL << 34);
    }
    if (keypad_debounced_state[9]) {
        new_keypad_bits |= (1ULL << 35);
    }
    keypad_buttons = new_keypad_bits;
    for (uint8_t i = 0; i < KEYPAD_BUTTON_COUNT; i++) {
        keypad_last_debounced_state[i] = keypad_debounced_state[i];
    }
}

int16_t get_wheel_position() {
    int16_t current_max = wheel_ranges[current_range_idx];
    int16_t pos = constrain(ffb_position, -current_max, current_max);
    return map(pos, -current_max, current_max, -32768, 32767);
}

int16_t get_accel_position() {
    uint16_t position = get_filtered_accel_raw();
    uint16_t c_min = min(hall_accel_min, hall_accel_max);
    uint16_t c_max = max(hall_accel_min, hall_accel_max);
    position = constrain(position, c_min, c_max);
    return map(position, hall_accel_min, hall_accel_max, -32768, 32767);
}

int16_t get_brake_position() {
    uint16_t position = get_filtered_brake_raw();
    uint16_t c_min = min(hall_brake_min, hall_brake_max);
    uint16_t c_max = max(hall_brake_min, hall_brake_max);
    position = constrain(position, c_min, c_max);
    return map(position, hall_brake_min, hall_brake_max, -32768, 32767);
}

void update_position() {
    ffb_last_position = ffb_position;
    ffb_last_position_time_us = ffb_position_time_us;
    ffb_position_time_us = micros();
    int16_t raw_pos;
    pcnt_get_counter_value(PCNT_UNIT_USED, &raw_pos);
    ffb_position = raw_pos - ffb_position_offset;
}

void hall_calib() {
    hall_accel_min = 0; hall_brake_min = 0;
    hall_accel_max = 4095; hall_brake_max = 4095;
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        if (hall_accel_val > hall_accel_min) {
            hall_accel_min = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        if (hall_brake_val > hall_brake_min) {
            hall_brake_min = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES_MS);
    }
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        if (hall_accel_val < hall_accel_max) {
            hall_accel_max = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        if (hall_brake_val < hall_brake_max) {
            hall_brake_max = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES_MS);
    }
    save_hall_calib_to_flash();
}

uint8_t calculate_crc_xor(const uint8_t* data, uint8_t len_without_crc) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len_without_crc; i++) {
        crc ^= data[i];
    }
    return crc;
}

void process_secondary_packet(uint8_t *packet) {
    uint8_t type = packet[1];
    uint8_t id = packet[2];
    uint16_t value = packet[3] | (packet[4] << 8);
    switch (type) {
        case MSG_TYPE_BUTTON:
            if (id < SECONDARY_BUTTON_COUNT) {
                const uint8_t bit_index = SECONDARY_BUTTON_OFFSET + id;
                if (value) {
                    secondary_buttons |= (1ULL << bit_index);
                } else {
                    secondary_buttons &= ~(1ULL << bit_index);
                }
            }
            break;
    }
}

void handle_secondary_uart() {
    while (Serial1.available()) {
        uint8_t byte_received = Serial1.read();
        if (rxIndex == 0) {
            if (byte_received == START_BYTE) {
                rxBuffer[rxIndex++] = byte_received;
            }
        } else {
            rxBuffer[rxIndex++] = byte_received;
            if (rxIndex >= PACKET_SIZE) {
                if (rxBuffer[5] == calculate_crc_xor(rxBuffer, 5)) {
                    process_secondary_packet(rxBuffer);
                }
                rxIndex = 0;
            }
        }
    }
}

uint8_t bridge_packet_len(uint8_t type) {
    if (type == BRIDGE_PKT_TORQUE_CMD) {
        return 8;
    }
    if (type == BRIDGE_PKT_CONTROL_CMD) {
        return 7;
    }
    return 0;
}

void process_bridge_command(uint8_t cmd, int16_t value) {
    switch (cmd) {
        case BRIDGE_CMD_STARTUP:
            bridge_startup_flags = (uint16_t)value;
            bridge_startup_received = true;
            break;
    }
}

void process_bridge_packet(const uint8_t* pkt, uint8_t len) {
    if (len < 4) {
        return;
    }
    const uint8_t type = pkt[1];
    if (type == BRIDGE_PKT_TORQUE_CMD && len >= 8) {
        int16_t torque = (int16_t)((uint16_t)pkt[3] | ((uint16_t)pkt[4] << 8));
        ffb_torque_value = constrain(torque, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
        ffb_device_gain = pkt[5];
        return;
    }
    if (type == BRIDGE_PKT_CONTROL_CMD && len >= 7) {
        process_bridge_command(pkt[3], (int16_t)((uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8)));
    }
}

void handle_bridge_serial() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        if (bridge_rx_idx == 0) {
            if (b == BRIDGE_START_BYTE) {
                bridge_rx_buf[bridge_rx_idx++] = b;
                bridge_rx_expected = 0;
            }
            continue;
        }
        if (bridge_rx_idx >= sizeof(bridge_rx_buf)) {
            bridge_rx_idx = 0; bridge_rx_expected = 0;
            continue;
        }
        bridge_rx_buf[bridge_rx_idx++] = b;
        if (bridge_rx_idx == 2) {
            bridge_rx_expected = bridge_packet_len(bridge_rx_buf[1]);
            if (bridge_rx_expected == 0 || bridge_rx_expected > sizeof(bridge_rx_buf)) {
                bridge_rx_idx = 0; bridge_rx_expected = 0;
            }
            continue;
        }
        if (bridge_rx_expected > 0 && bridge_rx_idx >= bridge_rx_expected) {
            if (bridge_rx_buf[bridge_rx_expected - 1] == calculate_crc_xor(bridge_rx_buf, bridge_rx_expected - 1)) {
                process_bridge_packet(bridge_rx_buf, bridge_rx_expected);
            }
            bridge_rx_idx = 0;
            bridge_rx_expected = 0;
        }
    }
}

void send_bridge_telemetry() {
    uint8_t pkt[18];
    pkt[0] = BRIDGE_START_BYTE;
    pkt[1] = BRIDGE_PKT_TELEMETRY;
    pkt[2] = 0; 
    pkt[3] = (uint8_t)(joyX & 0xFF);
    pkt[4] = (uint8_t)((joyX >> 8) & 0xFF);
    pkt[5] = (uint8_t)(joyY & 0xFF);
    pkt[6] = (uint8_t)((joyY >> 8) & 0xFF);
    pkt[7] = (uint8_t)(joyZ & 0xFF);
    pkt[8] = (uint8_t)((joyZ >> 8) & 0xFF);
    pkt[9]  = (uint8_t)(hid_buttons & 0xFF);
    pkt[10] = (uint8_t)((hid_buttons >> 8) & 0xFF);
    pkt[11] = (uint8_t)((hid_buttons >> 16) & 0xFF);
    pkt[12] = (uint8_t)((hid_buttons >> 24) & 0xFF);
    pkt[13] = (uint8_t)((hid_buttons >> 32) & 0xFF);
    pkt[14] = (uint8_t)((hid_buttons >> 40) & 0xFF);
    pkt[15] = (uint8_t)((hid_buttons >> 48) & 0xFF);
    pkt[16] = (uint8_t)((hid_buttons >> 56) & 0xFF);
    pkt[17] = calculate_crc_xor(pkt, 17);
    Serial.write(pkt, sizeof(pkt));
}

int16_t get_pos_PID(int32_t target_position) {
    uint32_t dt_us = (ffb_position_time_us - ffb_last_position_time_us);
    int32_t ffb_position_error = target_position - ffb_position;
    if (dt_us > 0) {
        ffb_position_integral += ffb_position_error * dt_us;
        ffb_position_integral = constrain(ffb_position_integral, -FFB_POS_I_LIMIT, FFB_POS_I_LIMIT);
        ffb_position_derivative = (float)(ffb_position_error - ffb_last_position_error) / (float)dt_us;
    } else {
        ffb_position_derivative = 0.0f;
    }
    ffb_last_position_error = ffb_position_error;
    return constrain(FFB_POS_KP * ffb_position_error + FFB_POS_KI * ffb_position_integral + FFB_POS_KD * ffb_position_derivative, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
}

void reset_position_pid_state() {
    ffb_position_integral = 0;
    ffb_last_position_error = 0;
    ffb_position_derivative = 0.0f;
}

void motor_init() {
    pinMode(BTS_REN_PIN, OUTPUT);
    pinMode(BTS_LEN_PIN, OUTPUT);
    digitalWrite(BTS_REN_PIN, HIGH);
    digitalWrite(BTS_LEN_PIN, HIGH);
    ledcSetup(BTS_PWM_CH_R, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    ledcSetup(BTS_PWM_CH_L, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    ledcAttachPin(BTS_RPWM_PIN, BTS_PWM_CH_R);
    ledcAttachPin(BTS_LPWM_PIN, BTS_PWM_CH_L);
    ledcWrite(BTS_PWM_CH_R, 0);
    ledcWrite(BTS_PWM_CH_L, 0);
}

void set_motor_torque(int16_t torque_val) {
    if (!ffb_software_enabled) {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, 0);
        return;
    }
    torque_val = constrain(torque_val, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
    int32_t scaled = (int32_t)torque_val * ffb_device_gain / 255;
    int pwm = map(abs((int)scaled), 0, FFB_ABS_MAX_VAL, 0, BTS_PWM_MAX);
    if (pwm < FFB_DEADZONE) {
        pwm = 0;
    }
    if (scaled > 0) {
        ledcWrite(BTS_PWM_CH_R, pwm);
        ledcWrite(BTS_PWM_CH_L, 0);
    } else if (scaled < 0) {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, pwm);
    } else {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, 0);
    }
}

void motor_self_test() {
    uint32_t start_time = millis();
    uint32_t hold_pos_time = 0;
    bool last_hold = false;
    int16_t target = -MOTOR_SELF_TEST_POSITION;
    reset_position_pid_state();
    for (int phase = 0; phase < 3; phase++) {
        if (phase == 1) {
            target = MOTOR_SELF_TEST_POSITION;
        }
        if (phase == 2) {
            target = 0;
        }
        start_time = millis(); hold_pos_time = 0; last_hold = false;
        reset_position_pid_state();
        while (true) {
            update_position();
            float pid_output = get_pos_PID(target);
            set_motor_torque((int16_t)constrain(pid_output, -MOTOR_SELF_TEST_TORQUE_LIMIT, MOTOR_SELF_TEST_TORQUE_LIMIT));
            if (abs(target - ffb_position) <= MOTOR_SELF_TEST_POS_TOL) {
                if (!last_hold) {
                    hold_pos_time = millis();
                    last_hold = true;
                } else if (millis() - hold_pos_time >= MOTOR_SELF_TEST_HOLD_TIME_MS) {
                    break;
                }
            } else {
                last_hold = false;
            }
            if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
                break;
            }
            delay(10);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!bridge_startup_received) {
        handle_bridge_serial();
        delay(1);
    }
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);
    keypad_init();
    encoder_init();
    hall_init();
    motor_init();

    init_pedal_filter();
    if (bridge_startup_flags & STARTUP_FLAG_AUTO_CALIB) {
        hall_calib();
    } else if (!load_hall_calib_from_flash()) { }
    if (!(bridge_startup_flags & STARTUP_FLAG_SKIP_SELF_TEST)) {
        motor_self_test();
    }
    int16_t initial_raw_pos = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &initial_raw_pos);
    ffb_position_offset = initial_raw_pos;
}

void loop() {
    update_keypad_buttons();
    handle_secondary_uart();
    hid_buttons = keypad_buttons | secondary_buttons;
    handle_bridge_serial();
    update_position();
    update_pedal_filter();
    joyX = get_wheel_position();
    joyY = get_accel_position();
    joyZ = get_brake_position();
    uint32_t now = millis();
    set_motor_torque(ffb_torque_value);
    if ((uint32_t)(now - last_bridge_telem_ms) >= BRIDGE_TELEMETRY_INTERVAL_MS) {
        last_bridge_telem_ms = now;
        send_bridge_telemetry();
    }
    delay(1);
}
