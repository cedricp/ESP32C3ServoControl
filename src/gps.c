#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include <string.h>
#include "gps.h"
#include "config.h"

QueueHandle_t gps_queue;

ubx_nav_pvt_t g_pvt_data;

inline void update_checksum(uint8_t cb, uint8_t *CK_A, uint8_t *CK_B) {
    *CK_A = *CK_A + cb;
    *CK_B = *CK_B + *CK_A;
}

static void gps_set_rate(uint16_t rate_ms) {
    uint8_t cfg_rate_msg[] = {
        0xB5, 0x62,         // Sync chars
        0x06, 0x08,         // Class: CFG, ID: RATE
        0x06, 0x00,         // Payload length (6 bytes)
        
        // Payload
        (uint8_t)(rate_ms & 0xFF),        // measRate (low byte)
        (uint8_t)((rate_ms >> 8) & 0xFF), // measRate (high byte)
        0x01, 0x00,                       // navRate = 1
        0x00, 0x00,                       // timeRef = 0 (UTC)
        
        0x00, 0x00          // Emplacement Checksum CK_A, CK_B
    };

    // Calcul du checksum UBX sur la classe, l'ID, la longueur et le payload (du byte 2 au byte 11)
    uint8_t CK_A = 0, CK_B = 0;
    for (int i = 2; i < 10; i++) {
        update_checksum(cfg_rate_msg[i], &CK_A, &CK_B);
    }

    cfg_rate_msg[12] = CK_A;
    cfg_rate_msg[13] = CK_B;

    uart_wait_tx_done(GPS_UART_PORT, pdMS_TO_TICKS(100));

    // Envoi de la commande de configuration via l'UART
    uart_write_bytes(GPS_UART_PORT, (const char *)cfg_rate_msg, sizeof(cfg_rate_msg));
}

void gps_task(void *pvParameters) {
    
    uint8_t byte;
    int state = 0;
    uint8_t msg_class = 0, msg_id = 0;
    uint16_t payload_length = 0;
    uint16_t payload_idx = 0;
    
    ubx_nav_pvt_t pvt_data;
    uint8_t *pvt_ptr = (uint8_t *)&pvt_data;
    uint8_t CK_A = 0, CK_B = 0;
    uint8_t rec_CK_A = 0, rec_CK_B = 0;

    // Read UBlox binary data from GPS
    while (1) {
        if (uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0) {
            switch (state) {
                case 0: // Wait Sync 1
                    if (byte == 0xB5) state = 1;
                    break;
                case 1: // Wait Sync 2
                    if (byte == 0x62) state = 2; else state = 0;
                    break;
                    
                case 2: // Mmessage class.
                    msg_class = byte;
                    CK_A = 0; CK_B = 0; // Checksum init
                    update_checksum(byte, &CK_A, &CK_B);
                    state = 3;
                    break;
                    
                case 3: // Mssage ID
                    msg_id = byte;
                    update_checksum(byte, &CK_A, &CK_B);
                    state = 4;
                    break;
                    
                case 4: // Payload length (Low byte)
                    payload_length = byte;
                    update_checksum(byte, &CK_A, &CK_B);
                    state = 5;
                    break;
                    
                case 5: // Payload length (Hi byte)
                    payload_length |= (byte << 8);
                    update_checksum(byte, &CK_A, &CK_B);
                    
                    if (msg_class == 0x01 && msg_id == 0x07) {
                        payload_idx = 0;
                        state = 6;
                    } else {
                        state = 0; 
                    }
                    break;
                    
                case 6: // Payload reading
                    pvt_ptr[payload_idx++] = byte;
                    update_checksum(byte, &CK_A, &CK_B);
                    
                    if (payload_idx >= payload_length) {
                        state = 7;
                    }
                    break;
                    
                case 7: // Checksum A
                    rec_CK_A = byte;
                    state = 8;
                    break;
                    
                case 8: // Checksum B
                    rec_CK_B = byte;
                    
                    // Checksum validation
                    if (CK_A == rec_CK_A && CK_B == rec_CK_B) {
                        if (pvt_data.fixType >= 3) {
                            xQueueSend(gps_queue, &pvt_data, 0);
                            memcpy(&g_pvt_data, &pvt_data, sizeof(ubx_nav_pvt_t));
                        } else {
                        }
                    } else {
                    }
                    state = 0; // Ready for next frame
                    break;
            }
        }
    }
}

void gps_init()
{
    gps_queue = xQueueCreate(1, sizeof(ubx_nav_pvt_t));
    
    uart_driver_install(GPS_UART_PORT, 1024, 256, 0, NULL, 0);
    // Init UART for GPS reception
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // gps_set_rate(2000);
}