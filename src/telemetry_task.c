#include "freertos/FreeRTOS.h"
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

extern struct system_data_t sysdata;

static int64_t last_gps_frame_time  = 0;
static int64_t last_esc_frame_time  = 0;
static battery_type_t g_battery_type = BATTERY_UNKNOWN;

#define KISS_FRAME_SIZE 10

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

static void switch_uart_rx_pin(gpio_num_t new_rx_pin, int rate)
{
    uart_config_t uart_config = {
        .baud_rate = rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_disable_rx_intr(GPS_UART_PORT);
    uart_set_pin(GPS_UART_PORT, GPIO_NUM_NC, new_rx_pin, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_flush_input(GPS_UART_PORT);
    uart_enable_rx_intr(GPS_UART_PORT);
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
    for (int i = 2; i < 12; i++) {
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
    uint8_t rx_buffer[64];
    esc_telemetry_t esc_telemetry_data;

    switch_uart_rx_pin(ESC_RX_GPIO, ESC_BITRATE);
    gpio_set_pull_mode(ESC_RX_GPIO, GPIO_PULLUP_ONLY);

    while(1)
    {
        int len = uart_read_bytes(GPS_UART_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(50));
        
        if (len > 0) 
        {
            for (int i = 0; i <= len - KISS_FRAME_SIZE; i++) 
            {
                if (parse_kiss_frame(&rx_buffer[i], &esc_telemetry_data)) 
                {
                    int64_t current_time = esp_timer_get_time();
                    if (current_time - last_esc_frame_time > 100000) // 100ms
                    {
                        uint8_t battery_percentage = 0;
                        if (g_battery_type == BATTERY_UNKNOWN)
                        {
                            g_battery_type = identify_battery_type(esc_telemetry_data.voltage_mv);
                        }
                        else
                        {
                            battery_percentage = get_battery_percentage(g_battery_type, esc_telemetry_data.voltage_mv);
                        }
                        
                        crsf_send_battery_packet(esc_telemetry_data.voltage_mv / 100, 
                                                 esc_telemetry_data.current_ma / 100, 
                                                 esc_telemetry_data.mah, 
                                                 battery_percentage);
                        crsf_send_temp(esc_telemetry_data.temperature * 10, 1);
                        crsf_send_rpm(esc_telemetry_data.erpm / (sysdata.motor_magnets_count / 2));
                        
                        last_esc_frame_time = current_time;
                        sysdata.esc_temperature = esc_telemetry_data.temperature;
                        return;
                    }
                }
            }
        }
        else
        {
            crsf_send_battery_packet(0, 0, 0, 0);
            crsf_send_temp(0, 1);
            crsf_send_rpm(0);
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
    ubx_nav_posllh_t posllh_data;
    uint8_t payload_buf[UBX_NAV_PVT_LEN];
    uint8_t CK_A = 0, CK_B = 0;
    uint8_t rec_CK_A = 0, rec_CK_B = 0;
    crsf_telemetry_gps_t crsf_gps_data;
    memset(&crsf_gps_data, 0, sizeof(crsf_telemetry_gps_t));

    switch_uart_rx_pin(GPS_RX_PIN, GPS_BAUD_RATE);
    while (1)
    {
        if (uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(200)) <= 0) {
            break;
        }

        switch (state) {
            case 0: // Sync 1
                if (byte == 0xB5) state = 1;
                break;

            case 1: // Sync 2
                state = (byte == 0x62) ? 2 : 0;
                break;

            case 2: // Class
                msg_class = byte;
                CK_A = 0; CK_B = 0;
                update_checksum(byte, &CK_A, &CK_B);
                state = 3;
                break;

            case 3: // ID
                msg_id = byte;
                update_checksum(byte, &CK_A, &CK_B);
                state = 4;
                break;

            case 4: // Length low
                payload_length = byte;
                update_checksum(byte, &CK_A, &CK_B);
                state = 5;
                break;

            case 5: // Length high
                payload_length |= (byte << 8);
                update_checksum(byte, &CK_A, &CK_B);

                if (msg_class == 0x01 &&
                    ((msg_id == 0x02 && payload_length == UBX_NAV_POSLLH_LEN) ||
                     (msg_id == 0x07 && payload_length == UBX_NAV_PVT_LEN))) {
                    payload_idx = 0;
                    state = 6;
                } else {
                    state = 0;
                }
                break;

            case 6: // Payload
                payload_buf[payload_idx++] = byte;
                update_checksum(byte, &CK_A, &CK_B);
                if (payload_idx >= payload_length) state = 7;
                break;

            case 7: // Checksum A
                rec_CK_A = byte;
                state = 8;
                break;

            case 8: // Checksum B
                rec_CK_B = byte;

                if (CK_A == rec_CK_A && CK_B == rec_CK_B) {
                    if (msg_id == 0x07) {
                        memcpy(&pvt_data, payload_buf, sizeof(pvt_data));
                    } else { // 0x02 NAV-POSLLH
                        memcpy(&posllh_data, payload_buf, sizeof(posllh_data));
                        memset(&pvt_data, 0, sizeof(pvt_data));
                        // fixType, numSV, gSpeed, headMot... stay at 0
                        pvt_data.iTOW   = posllh_data.iTOW;
                        pvt_data.lon    = posllh_data.lon;
                        pvt_data.lat    = posllh_data.lat;
                        pvt_data.height = posllh_data.height;
                        pvt_data.hMSL   = posllh_data.hMSL;
                        pvt_data.hAcc   = posllh_data.hAcc;
                        pvt_data.vAcc   = posllh_data.vAcc;
                    }

                    // printf("GPS UBX 0x%02X: Lat: %.7f, Lon: %.7f, Alt: %.2f m, FixType: %d, NumSV: %d\n",
                    //        msg_id, pvt_data.lat / 1e7, pvt_data.lon / 1e7,
                    //        pvt_data.height / 1000.0, pvt_data.fixType, pvt_data.numSV);

                    if (esp_timer_get_time() - last_gps_frame_time > 100000) {
                        forward_gps_to_elrs(&pvt_data, &crsf_gps_data);
                        crsf_send_gps_packet(&crsf_gps_data);
                        last_gps_frame_time = esp_timer_get_time();
                    }
                }
                return;
        }
    }
}

const char* flightmode_to_str(int flightmode_id) {
    if (sysdata.gyro_calibrated == 0) {
        return "UNCAL";
    }
    switch (flightmode_id) {
        case FLIGHTMODE_FREE: return "FREE";
        case FLIGHTMODE_STAB: return "STAB";
        case FLIGHTMODE_LEVEL: return "LVL";
        default: return "NA";
    }
}

static void process_attitude()
{
    crsf_send_attitude((int16_t)(deg_to_rad(sysdata.attitude.pitch_deg + sysdata.attitude_correction_rp[1]) * 10000.0f),
                       (int16_t)(deg_to_rad(sysdata.attitude.roll_deg  + sysdata.attitude_correction_rp[0]) * 10000.0f),
                       (int16_t)(deg_to_rad(sysdata.attitude.yaw_deg) * 10000.0f));
}

static void process_gyro_temperature()
{
    crsf_send_temp(sysdata.gyro_temperature, 0);
}

void telemetry_task(void *pvParameters)
{
    chrono_t chrono;
    chrono_start(&chrono);

    while (1) {
        process_esc();
        process_gps();
        if (chrono_elapsed_us(&chrono) > 200000)
        {
            const char *flightmode_str = flightmode_to_str(sysdata.flightmode);
            crsf_send_flightmode(flightmode_str);
            process_attitude();
            process_gyro_temperature();
            
            chrono_start(&chrono);
        }
    }
}

void gps_init()
{
    uart_driver_install(GPS_UART_PORT, 1024, 256, 0, NULL, 0);
    // Init UART for GPS/ESC reception
    // Both will share the same UART, but we will switch the RX pin depending on the mode (GPS or ESC)
    gpio_set_pull_mode(ESC_RX_GPIO, GPIO_PULLUP_ONLY);
    switch_uart_rx_pin(GPS_RX_PIN, GPS_BAUD_RATE);
}