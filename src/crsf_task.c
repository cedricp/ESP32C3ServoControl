#include "crsf_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "types.h"
#include "utils.h"
#include <string.h>

portMUX_TYPE g_servo_spinlock = portMUX_INITIALIZER_UNLOCKED;
servo_data_t g_servo_data;

void get_servo_data(servo_data_t *data)
{
    portENTER_CRITICAL(&g_servo_spinlock);
    *data = g_servo_data;
    portEXIT_CRITICAL(&g_servo_spinlock);
}

// ==========================================
// CROSSFIRE UART task
// ==========================================
void crsf_rx_task(void *pvParameters) {
    uint8_t buffer[128];   // Larger than one frame — holds overlap
    int buf_len = 0;
    
    // Init UART for CRSF reception
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(CRSF_UART_PORT, &uart_config);
    uart_set_pin(CRSF_UART_PORT, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CRSF_UART_PORT, 1024, 0, 0, NULL, 0);

    TickType_t last_rx_time = xTaskGetTickCount();

    while (1) {
        // Fill whatever space is left in the buffer
        int bytes_read = uart_read_bytes(
            CRSF_UART_PORT,
            buffer + buf_len,
            sizeof(buffer) - buf_len,
            pdMS_TO_TICKS(20)
        );

        if (bytes_read == 0 && (xTaskGetTickCount() - last_rx_time) > pdMS_TO_TICKS(CRSF_TIMEOUT_MS)) {
            // No data received for 250ms, send failsafe values to servo task
            portENTER_CRITICAL(&g_servo_spinlock);
            for (int i = 0; i < NUM_SERVOS; i++) {
                g_servo_data.us_values[i] = 1500;
            }
            g_servo_data.valid = 0;
            portEXIT_CRITICAL(&g_servo_spinlock);
        } else {
            last_rx_time = xTaskGetTickCount();
        }
        
        if (bytes_read > 0)
            buf_len += bytes_read;

        // Scan forward for a valid frame start
        int i = 0;
        while (i < buf_len) {

            // Step 1: find sync byte
            if (buffer[i] != 0xC8) {
                i++;
                continue;
            }

            // Step 2: do we have enough bytes to read the length field?
            if (i + 1 >= buf_len)
                break;  // Wait for more data

            uint8_t packet_len = buffer[i + 1];

            if (packet_len > 62) {
                // Implausible length — this 0xC8 was a false positive, keep scanning
                i++;
                continue;
            }

            // Step 3: do we have the full frame yet?  (sync + len + payload)
            int frame_end = i + 2 + packet_len;  // index of last byte + 1
            if (frame_end > buf_len)
                break;  // Partial frame — wait for more data, do NOT discard

            // Step 4: check frame type
            if (buffer[i + 2] != 0x16) {
                i++;  // False positive sync byte, keep scanning
                continue;
            }

            // Step 5: CRC check
            uint8_t computed_crc = crsf_crc8(&buffer[i + 2], packet_len - 1);
            if (computed_crc != buffer[frame_end - 1]) {
                i++;  // CRC failed — this sync byte was not a real frame start
                continue;
            }

            // --- Valid frame found at offset i ---
            uint8_t *payload = &buffer[i + 3];

            servo_data_t tx_data;
            tx_data.valid = 1;
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                tx_data.us_values[ch] = ((crsf_get_channel(ch, payload) - 992) * 3) / 5 + 1500;
            }
            portENTER_CRITICAL(&g_servo_spinlock);
            g_servo_data = tx_data;
            portEXIT_CRITICAL(&g_servo_spinlock);

            // Advance past the consumed frame
            i = frame_end;
        }

        // Shift remaining unprocessed bytes to the front of the buffer
        if (i > 0) {
            buf_len -= i;
            if (buf_len > 0)
                memmove(buffer, buffer + i, buf_len);
        }
    }
}