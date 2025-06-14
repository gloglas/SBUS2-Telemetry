/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(ESP32)

#include "SBUS_usart.h"
#include "esp32-hal.h"
#include "esp32-hal-uart.h"
#include "driver/uart.h"
#include <soc/uart_reg.h>
#include <driver/timer.h>
#include <soc/uart_struct.h>
#include <soc/timer_group_struct.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define SLOT_DATA_LENGTH         3
#define NUMBER_OF_FRAMES         4
#define NUMBER_OF_SLOT           32
#define NUMBER_OF_SLOT_IN_FRAME  8
#define SBUS_FRAME_SIZE          25

#if CONFIG_IDF_TARGET_ESP32
  #define SBUS2_UART_RX_PIN      (GPIO_NUM_25)
  #define SBUS2_UART_TX_PIN      (GPIO_NUM_26)
#elif CONFIG_IDF_TARGET_ESP32S2
  #define SBUS2_UART_RX_PIN      (2)
  #define SBUS2_UART_TX_PIN      (7)
#endif

#define UART_RXBUFSIZE           30
#define SLOT_TIME                660    // microseconds
#define WAIT_TIME                2000   // microseconds
#define RX_BUF_SIZE              1024   // UART driver buffer

#ifndef NUMBER_OF_CHANNELS
  #define NUMBER_OF_CHANNELS     18
#endif

bool DemoMode = false;

// Slot IDs for telemetry
uint8_t Slot_ID[NUMBER_OF_SLOT] = {
  0x03,0x83,0x43,0xC3,0x23,0xA3,0x63,0xE3,
  0x13,0x93,0x53,0xD3,0x33,0xB3,0x73,0xF3,
  0x0B,0x8B,0x4B,0xCB,0x2B,0xAB,0x6B,0xEB,
  0x1B,0x9B,0x5B,0xDB,0x3B,0xBB,0x7B,0xFB
};

// Demo frame for simulation mode
char sbus_frame[SBUS_FRAME_SIZE] = {
  0x0F,0xA0,0xA3,0x20,0x56,0x2C,0x08,0x16,
  0x50,0x03,0x10,0x80,0x00,0x04,0x20,0x00,
  0x01,0x08,0x07,0xC8,0x03,0x10,0x80,0x02,0x04
};

// Parsed channel values
uint16_t channels[NUMBER_OF_CHANNELS];

// Raw RX buffer and parsed SBUS data
static volatile uint8_t rxbuf[UART_RXBUFSIZE];
static volatile uint8_t sbusData[SBUS_FRAME_SIZE];
static volatile uint8_t telemetryData[256];

// Flags
static volatile bool sbus_ready      = false;
static volatile bool telemetry_ready = false;

// Telemetry transmit state
static volatile uint8_t gl_current_frame = 0;
static volatile bool transmit            = false;
static volatile uint8_t sequence_count   = 0;
volatile SLOT_DATA_STATUS transmit_data_per_slot_status[NUMBER_OF_SLOT];
volatile uint8_t transmit_data_per_slot_data[NUMBER_OF_SLOT][SLOT_DATA_LENGTH];

// Error/failsafe tracking
static volatile uint16_t uart_lost_frame = 0;
static volatile uint8_t  FER_count       = 0;
static volatile uint8_t  FER_buf[100];

// UART + timer state
static volatile uint8_t  buffer_index    = 0;
static volatile uint8_t  tx_pin;
uart_port_t              uart_num;

static volatile int8_t   previousFrame   = -1;
static volatile uint32_t frameCounter    = 0;

// --- Deferred‐work flags for SBUS frame parsing ---
static volatile bool     frame_received  = false;
static volatile uint8_t  frame_number    = 0;

// UART & timer configs
uart_config_t uart_config = {
    .baud_rate    = 100000,
    .data_bits    = UART_DATA_8_BITS,
    .parity       = UART_PARITY_EVEN,
    .stop_bits    = UART_STOP_BITS_2,
    .flow_ctrl    = UART_HW_FLOWCTRL_DISABLE
};

timer_config_t timer_config = {
    .alarm_en      = TIMER_ALARM_EN,
    .counter_en    = TIMER_PAUSE,
    .intr_type     = TIMER_INTR_LEVEL,
    .counter_dir   = TIMER_COUNT_UP,
    .auto_reload   = TIMER_AUTORELOAD_EN,
    .divider       = 80       // 1 µs per tick
};

// Forward declarations
void initialize_slot_status();
void enable_receiving();
void disable_receiving();
void start_transmit_sequencer(uint8_t frame_number);
void SBUS2_get_all_servo_data();

// Minimal UART ISR
static void IRAM_ATTR uart_intr_handle(void *arg)
{
    // Clear RX interrupts
    uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);

    // How many bytes are waiting?
    uint16_t len = UART1.status.rxfifo_cnt;
    // If it's not 0, 3, or 25 bytes, reset and resync
    if (len != 0 && len != SBUS_FRAME_SIZE && len != SLOT_DATA_LENGTH) {
        buffer_index = 0;
        return;
    }

    // Read them all
    while (len--) {
        uint8_t b = UART1.fifo.rw_byte;

        // Sync on frame header
        if (buffer_index == 0) {
            if (b != 0x0F) {
                continue;
            }
        }

        rxbuf[buffer_index++] = b;

        // Full SBUS frame?
        if (buffer_index == SBUS_FRAME_SIZE) {
            // Copy only the 25-byte frame
            memcpy((void*)sbusData, (const void*)rxbuf, SBUS_FRAME_SIZE);
            buffer_index = 0;

            // Defer heavy work
            frame_number   = (sbusData[24] & 0x30) >> 4;
            frame_received = true;
            return;
        }

        // Full telemetry slot?
        if (buffer_index == SLOT_DATA_LENGTH &&
            (((rxbuf[0] & 0x0F) == 0x03) || ((rxbuf[0] & 0x0F) == 0x0B)))
        {
            uint8_t id = rxbuf[0];
            telemetryData[id]     = rxbuf[0];
            telemetryData[id + 1] = rxbuf[1];
            telemetryData[id + 2] = rxbuf[2];
            buffer_index = 0;
            return;
        }

        // Slot length but invalid ID
        if (buffer_index == SLOT_DATA_LENGTH) {
            buffer_index = 0;
            return;
        }

        // Overflow guard
        if (buffer_index > SBUS_FRAME_SIZE) {
            buffer_index = 0;
            return;
        }
    }
}

// Timer ISR for telemetry‐slot transmission (unchanged timing, no digitalWrite)
static void IRAM_ATTR ISR_transmit_frame(void *arg)
{
#if CONFIG_IDF_TARGET_ESP32
    TIMERG0.int_clr_timers.t1 = 1;
    TIMERG0.hw_timer[1].config.alarm_en = 1;
#elif CONFIG_IDF_TARGET_ESP32S2
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    TIMERG0.hw_timer[1].config.tx_alarm_en = 1;
#endif

    // Detach TX until needed
    pinMatrixOutDetach(tx_pin, false, true);

    if (sequence_count < NUMBER_OF_SLOT_IN_FRAME) {
        uint8_t actual_slot = sequence_count + gl_current_frame * NUMBER_OF_SLOT_IN_FRAME;
        if (transmit_data_per_slot_status[actual_slot] == AVAILABLE) {
            pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);
            char buf[SLOT_DATA_LENGTH];
            memcpy(buf, transmit_data_per_slot_data[actual_slot], SLOT_DATA_LENGTH);
            transmit_data_per_slot_status[actual_slot] = TRANSMITTING;
            uart_write_bytes(uart_num, buf, SLOT_DATA_LENGTH);
            transmit_data_per_slot_status[actual_slot] = EMPTY;
        }
        // re-schedule next slot
        timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
        timer_set_alarm_value(
          TIMER_GROUP_0, TIMER_1,
          (sequence_count == NUMBER_OF_SLOT_IN_FRAME - 1) ? (SLOT_TIME * 8 + SLOT_TIME) : SLOT_TIME
        );
        timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);
    }
    else {
        // send SBUS2 frame (demo or idle)
        if (DemoMode) {
            pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);
            uart_write_bytes(uart_num, sbus_frame, SBUS_FRAME_SIZE);
            sbus_frame[24] += 0x10;
            if (sbus_frame[24] > 0x34) sbus_frame[24] = 0x04;
        }
        sequence_count = 0;
        transmit       = false;
        enable_receiving();
    }

    sequence_count++;
}

// Correct frame‐loss math
inline void IncreaseTimer(int8_t frameNum)
{
    uint8_t delta = (frameNum + NUMBER_OF_FRAMES - previousFrame) % NUMBER_OF_FRAMES;
    if (delta > 1) uart_lost_frame++;
    frameCounter += delta;
    previousFrame = frameNum;
}

// Must be called regularly (e.g. in loop()) to finish processing each received frame
void SBUS2_handle_frame()
{
    if (!frame_received) return;
    frame_received = false;

    // 1) Stop reception until we finish
    disable_receiving();

    // 2) Update timing & stats
    IncreaseTimer(frame_number);

    // 3) Parse channels
    SBUS2_get_all_servo_data();

    // 4) Decide telemetry vs plain SBUS
    if ((sbusData[24] & 0x0F) == 0x04) {
        telemetry_ready = true;
        sbus_ready      = true;
        gl_current_frame = frame_number;
        start_transmit_sequencer(frame_number);
    }
    else {
        telemetry_ready = false;
        sbus_ready      = true;
    }
}

// ————————————————————————————————————————————————————————————
// Remaining API functions are essentially unchanged, except that
// SBUS_Ready() now calls SBUS2_handle_frame() to finish ISR work.
// ————————————————————————————————————————————————————————————

void initialize_slot_status()
{
    for (uint8_t i = 0; i < NUMBER_OF_SLOT; i++)
        transmit_data_per_slot_status[i] = EMPTY;
}

void disable_receiving()
{
    sequence_count = 0;
    timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_DIS);
    timer_pause(TIMER_GROUP_0, TIMER_1);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, WAIT_TIME);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);
    timer_start(TIMER_GROUP_0, TIMER_1);
}

void enable_receiving()
{
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 15000);
    timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);
    buffer_index = 0;
}

void start_transmit_sequencer(uint8_t frame_number)
{
    gl_current_frame = frame_number;
    disable_receiving();
}

void SBUS2_transmit_telemetry_data(uint8_t slot, uint8_t data[SLOT_DATA_LENGTH])
{
    if (transmit_data_per_slot_status[slot] != TRANSMITTING) {
        transmit_data_per_slot_data[slot][0] = Slot_ID[slot];
        transmit_data_per_slot_data[slot][1] = data[1];
        transmit_data_per_slot_data[slot][2] = data[2];
        transmit_data_per_slot_status[slot]  = AVAILABLE;
    }
}

void SBUS2_get_all_servo_data()
{
    // Unchanged bit‐unpacking from sbusData to channels[]
    channels[0]  = ((sbusData[1] | sbusData[2]  << 8) & 0x07FF);
    channels[1]  = ((sbusData[2] >> 3 | sbusData[3]  << 5) & 0x07FF);
    channels[2]  = ((sbusData[3] >> 6 | sbusData[4]  << 2 | sbusData[5] << 10) & 0x07FF);
    channels[3]  = ((sbusData[5] >> 1 | sbusData[6]  << 7) & 0x07FF);
    channels[4]  = ((sbusData[6] >> 4 | sbusData[7]  << 4) & 0x07FF);
    channels[5]  = ((sbusData[7] >> 7 | sbusData[8]  << 1 | sbusData[9] << 9) & 0x07FF);
    channels[6]  = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
    channels[7]  = ((sbusData[10]>> 5 | sbusData[11] << 3) & 0x07FF);
    channels[8]  = ((sbusData[12] | sbusData[13] << 8) & 0x07FF);
    channels[9]  = ((sbusData[13]>> 3 | sbusData[14] << 5) & 0x07FF);
    channels[10] = ((sbusData[14]>> 6 | sbusData[15] << 2 | sbusData[16] <<10) & 0x07FF);
    channels[11] = ((sbusData[16]>> 1 | sbusData[17] << 7) & 0x07FF);
    channels[12] = ((sbusData[17]>> 4 | sbusData[18] << 4) & 0x07FF);
    channels[13] = ((sbusData[18]>> 7 | sbusData[19] << 1 | sbusData[20] <<9) & 0x07FF);
    channels[14] = ((sbusData[20]>> 2 | sbusData[21] << 6) & 0x07FF);
    channels[15] = ((sbusData[21]>> 5 | sbusData[22] << 3) & 0x07FF);
    channels[16] = (sbusData[23] & (1 << 0)) ? 1 : 0;
    channels[17] = (sbusData[23] & (1 << 1)) ? 1 : 0;

    FER_buf[FER_count] = (sbusData[23] & 0x04) ? 1 : 0;
    if (++FER_count > 99) FER_count = 0;
}

void SBUS2_get_status(uint16_t *uart_dropped_frame,
                      bool    *transmission_dropped_frame,
                      bool    *failsafe)
{
    if (frameCounter < 60) uart_lost_frame = 0;
    *uart_dropped_frame        = uart_lost_frame;
    uart_lost_frame            = 0;
    *transmission_dropped_frame= (sbusData[23] & 0x04);
    *failsafe                  = (sbusData[23] & 0x08);
}

int16_t SBUS2_get_servo_data(uint8_t channel)
{
    return (channel < NUMBER_OF_CHANNELS) ? channels[channel] : -1;
}

bool SBUS_Ready()
{
    SBUS2_handle_frame();
    if (sbus_ready) {
        sbus_ready = false;
        return true;
    }
    return false;
}

bool SBUS2_Ready()
{
    if (telemetry_ready) {
        telemetry_ready = false;
        return true;
    }
    return false;
}

bool SBUS_Ready(bool reset)
{
    SBUS2_handle_frame();
    if (sbus_ready) {
        if (reset) sbus_ready = false;
        return true;
    }
    return false;
}

bool SBUS2_Ready(bool reset)
{
    if (telemetry_ready) {
        if (reset) telemetry_ready = false;
        return true;
    }
    return false;
}

uint8_t SBUS_get_FER()
{
    uint8_t fer = 0;
    for (uint8_t i = 0; i < 100; i++)
        if (FER_buf[i]) fer++;
    return fer;
}

uint8_t SBUS_get_RSSI()
{
    return 100 - SBUS_get_FER();
}

void SBUS2_print_Raw()
{
    for (uint16_t i = 0; i < 256; i++) {
        Serial.print(i, HEX); Serial.print(' ');
    }
    Serial.println();
    for (uint16_t i = 0; i < 256; i++) {
        Serial.print(telemetryData[i], HEX); Serial.print(' ');
    }
    Serial.println();
}

bool SBUS2_get_Slot(uint8_t slot, uint8_t *lowbyte, uint8_t *highbyte)
{
    uint8_t id = Slot_ID[slot];
    if (telemetryData[id] == id) {
        *lowbyte  = telemetryData[id + 1];
        *highbyte = telemetryData[id + 2];
        telemetryData[id]     = 0;
        telemetryData[id + 1] = 0;
        telemetryData[id + 2] = 0;
        return true;
    }
    *lowbyte  = 0;
    *highbyte = 0;
    return false;
}

void SBUS_disable()
{
    uart_disable_rx_intr(uart_num);
    timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_DIS);
}

void SBUS_enable()
{
    uart_enable_rx_intr(uart_num);
    initialize_slot_status();
    enable_receiving();
}

void SBUS2_disable()
{
    SBUS_disable();
}

void SBUS2_enable()
{
    SBUS_enable();
}

#endif // defined(ESP32)
