#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include <string.h>
#include "telemetry_task.h"
#include "crsf_types.h"
#include "crsf_task.h"
#include "config.h"
#include "utils.h"
#include "pid.h"

extern volatile uint32_t g_esc_temperature;
extern uint16_t    g_motor_magnets_count;
extern attitude_t  g_attitude;
extern float       g_attitude_correction_rp[2];

uint32_t last_gps_frame_time = 0;
uint32_t last_esc_frame_time = 0;
static battery_type_t g_battery_type = BATTERY_UNKNOWN;

inline void update_checksum(uint8_t cb, uint8_t *CK_A, uint8_t *CK_B) {
    *CK_A = *CK_A + cb;
    *CK_B = *CK_B + *CK_A;
}

static bool parse_kiss_frame(const uint8_t *frame, esc_telemetry_t *data) {
    uint8_t computed_crc = calculate_crc8_kiss(frame, 9);
    if (computed_crc != frame[9]) {
        return false;
    }

    data->temperature    = frame[0];
    data->voltage_mv     = (float)((frame[1] << 8) | frame[2]) * 10.0f; 
    data->current_ma     = (float)((frame[3] << 8) | frame[4]) * 10.0f; 
    data->mah            = (frame[5] << 8) | frame[6];
    data->erpm           = (uint32_t)((frame[7] << 8) | frame[8]) * 100;

    return true;
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

static void process_esc()
{
    uint8_t byte;
    uint8_t frame[20];
    uint8_t frame_idx = 0;
    esc_telemetry_t esc_telemetry_data;

    uart_set_pin(GPS_UART_PORT, GPIO_NUM_NC, ESC_RX_GPIO, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_flush_input(GPS_UART_PORT);
    bool kiss_frame_received = false;
    while(1)
    {
        if (uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(200)) > 0) 
        {
            frame[frame_idx++] = byte;
            
            if (frame_idx >= sizeof(frame)) {
                frame_idx = 0; // Reset if overflow
            }
            
            if (frame_idx >= 10) {
                int offset = frame_idx - 10;
                if (parse_kiss_frame(frame + offset, &esc_telemetry_data)) {
                    kiss_frame_received = true;
                    if (esp_timer_get_time() - last_esc_frame_time > 100000) { // 100ms
                        uint8_t battery_percentage = 0;
                        if (g_battery_type == BATTERY_UNKNOWN)
                        {
                            g_battery_type = identifyBatteryType(esc_telemetry_data.voltage_mv);
                        }
                        else
                        {
                            battery_percentage = calcBatteryPercentage(g_battery_type, esc_telemetry_data.voltage_mv);
                        }
                        crsf_send_battery_packet(esc_telemetry_data.voltage_mv / 100, esc_telemetry_data.current_ma / 100, esc_telemetry_data.mah, battery_percentage);
                        crsf_send_temp(esc_telemetry_data.temperature*10);
                        crsf_send_rpm(esc_telemetry_data.erpm/(g_motor_magnets_count/2));
                        last_esc_frame_time = esp_timer_get_time();
                        return;
                    }
                }
            }
        }
        else
        {
            break;
        }
    }
}

static void process_gps()
{
    uint8_t byte;
    int state = 0;
    uint8_t msg_class = 0, msg_id = 0;
    uint16_t payload_length = 0;
    uint16_t payload_idx = 0;
    
    ubx_nav_pvt_t pvt_data;
    uint8_t *pvt_ptr = (uint8_t *)&pvt_data;
    uint8_t CK_A = 0, CK_B = 0;
    uint8_t rec_CK_A = 0, rec_CK_B = 0;
    bool data_received = false;
    crsf_telemetry_gps_t crsf_gps_data;

    uart_set_pin(GPS_UART_PORT, GPIO_NUM_NC, GPS_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_flush_input(GPS_UART_PORT);
    while(1)
    {
        if (uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(200)) > 0) 
        {
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
                        data_received = true;
                        // printf("GPS UBX PVT received: Time: %04d-%02d-%02d %02d:%02d:%02d, Lat: %.7f, Lon: %.7f, Alt: %.2f m, FixType: %d, NumSV: %d\n",
                        //         pvt_data.year, pvt_data.month, pvt_data.day,
                        //         pvt_data.hour, pvt_data.min, pvt_data.sec,
                        //         pvt_data.lat / 1e7, pvt_data.lon / 1e7,
                        //         pvt_data.height / 1000.0, pvt_data.fixType,
                        //         pvt_data.numSV);
                        if (esp_timer_get_time() - last_gps_frame_time > 100000) { // 100ms
                            forward_gps_to_elrs(&pvt_data, &crsf_gps_data);
                            crsf_send_gps_packet(&crsf_gps_data); 
                            last_gps_frame_time = esp_timer_get_time();
                        }
                    }
                    return;
            }
        }
        else
        {
            break;
        }
        if (data_received)
        {
            break;
        }
    }
}

static void process_attitude()
{
    crsf_send_attitude((int16_t)(g_attitude.pitchDeg * (M_PI / 180.0f) * 10000.0f),
                       (int16_t)(g_attitude.rollDeg * (M_PI / 180.0f) * 10000.0f),
                       0);
}

void telemetry_task(void *pvParameters)
{
    while (1) {
        process_esc();
        process_gps();
        process_attitude();
    }
}

void gps_init()
{
    uart_driver_install(GPS_UART_PORT, 1024, 256, 0, NULL, 0);
    // Init UART for GPS/ESC reception
    // Both will share the same UART, but we will switch the RX pin depending on the mode (GPS or ESC)
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}