#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include <string.h>
#include "esp_attr.h"

#include "crsf_task.h"
#include "config.h"

SemaphoreHandle_t g_crsf_mutex = NULL;
extern uint16_t   g_motor_magnets_count;

servo_data_t g_servo_data;

IRAM_ATTR void get_servo_data(servo_data_t *data)
{
    if (xSemaphoreTake(g_crsf_mutex, pdMS_TO_TICKS(5)) == pdTRUE) 
    {
        *data = g_servo_data;
        xSemaphoreGive(g_crsf_mutex);
    }
}

static inline uint16_t __attribute__((always_inline)) crsf_get_channel(int ch, const uint8_t *payload)
{

    int bit_offset = ch * 11;
    int byte_index = bit_offset >> 3;  // bit_offset / 8
    int bit_shift  = bit_offset & 0x07; // bit_offset % 8

    // Read 3 bytes to guarantee 11 bits are always available regardless of alignment
   uint32_t raw = (uint32_t)payload[byte_index] | ((uint32_t)payload[byte_index + 1] << 8);

    if (byte_index + 2 < 22)
    {
        raw |= ((uint32_t)payload[byte_index + 2] << 16);
    }

    return (raw >> bit_shift) & 0x07FF;
}

void crsf_init()
{
    g_crsf_mutex = xSemaphoreCreateMutex();

    uart_driver_install(CRSF_UART_PORT, 1024, 512, 0, NULL, 0);
    
    // Init UART for CRSF reception
    uart_config_t uart_config = {
        .baud_rate  = CRSF_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(CRSF_UART_PORT, &uart_config);
    uart_set_pin(CRSF_UART_PORT, CRSF_TX_PIN, CRSF_RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC);
}

// ==========================================
// CROSSFIRE UART task
// ==========================================
void crsf_task_rx(void *pvParameters)
{
    uint8_t buffer[128]; // Larger than one frame — holds overlap
    int buf_len = 0;

    TickType_t last_rx_time = xTaskGetTickCount();

    esp_task_wdt_add(NULL);

    while (1)
    {
        esp_task_wdt_reset();
        // Fill whatever space is left in the buffer
        int bytes_read = uart_read_bytes(
            CRSF_UART_PORT,
            buffer + buf_len,
            sizeof(buffer) - buf_len,
            pdMS_TO_TICKS(20));

        if (bytes_read == 0 && (xTaskGetTickCount() - last_rx_time) > pdMS_TO_TICKS(CRSF_TIMEOUT_MS))
        {
            // No data received for 250ms, send failsafe values to servo task
            if (xSemaphoreTake(g_crsf_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_servo_data.valid = 0;
                xSemaphoreGive(g_crsf_mutex);
            }
        }
        else if (bytes_read > 0)
        {
            last_rx_time = xTaskGetTickCount(); // ← Only update when data arrives
        }

        if (bytes_read > 0)
            buf_len += bytes_read;

        // Scan forward for a valid frame start
        int i = 0;
        while (i < buf_len)
        {

            // Step 1: find sync byte
            if (buffer[i] != 0xC8)
            {
                i++;
                continue;
            }

            // Step 2: do we have enough bytes to read the length field?
            if (i + 1 >= buf_len)
                break; // Wait for more data

            uint8_t packet_len = buffer[i + 1];

            if (packet_len < 4 || packet_len > 62)
            {
                // Implausible length — this 0xC8 was a false positive, keep scanning
                i++;
                continue;
            }

            // Step 3: do we have the full frame yet?  (sync + len + payload)
            int frame_end = i + 2 + packet_len; // index of last byte + 1
            if (frame_end > buf_len)
                break; // Partial frame — wait for more data, do NOT discard

            // Step 4: check frame type
            if (buffer[i + 2] != 0x16)
            {
                i++; // False positive sync byte, keep scanning
                continue;
            }

            // Step 5: CRC check
            uint8_t computed_crc = calculate_crc8_crsf(&buffer[i + 2], packet_len - 1);
            if (computed_crc != buffer[frame_end - 1])
            {
                i++; // CRC failed — this sync byte was not a real frame start
                continue;
            }

            // --- Valid frame found at offset i ---
            uint8_t *payload = &buffer[i + 3];

            servo_data_t tx_data;
            tx_data.valid = 1;
            for (int ch = 0; ch < NUM_CRSF_CHANNELS; ch++)
            {
                tx_data.us_values[ch] = ((crsf_get_channel(ch, payload) - 992) * 3) / 5 + 1500;
            }
            if (xSemaphoreTake(g_crsf_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_servo_data = tx_data;
                xSemaphoreGive(g_crsf_mutex);
            }

            // Advance past the consumed frame
            i = frame_end;
        }

        if (i == 0 && buf_len == sizeof(buffer))
        {
            i = 1;
        }

        // Shift remaining unprocessed bytes to the front of the buffer
        if (i > 0)
        {
            buf_len -= i;
            if (buf_len > 0)
                memmove(buffer, buffer + i, buf_len);
        }
    }
}

void send_crsf_volt_array(uint16_t batt1_mv, uint16_t batt2_mv) {
    uint8_t tx_buffer[9];

    tx_buffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xC8
    tx_buffer[1] = 7;                              // Length
    tx_buffer[2] = CRSF_FRAMETYPE_CELLS_SENSOR;    // 0x0E
    
    tx_buffer[3] = 0x80;                           // sensorID >= 128 (ex: 0x80)

    uint16_t val1 = batt1_mv * 10;
    uint16_t val2 = batt2_mv * 10;

    // Big-Endian
    tx_buffer[4] = (uint8_t)((val1 >> 8) & 0xFF);
    tx_buffer[5] = (uint8_t)(val1 & 0xFF);
    
    tx_buffer[6] = (uint8_t)((val2 >> 8) & 0xFF);
    tx_buffer[7] = (uint8_t)(val2 & 0xFF);

    tx_buffer[8] = calculate_crc8_crsf(&tx_buffer[2], 6);

    uart_write_bytes(CRSF_UART_PORT, (const char *)tx_buffer, sizeof(tx_buffer));
}

// Function to forward data without floating-point emulation
void forward_gps_to_elrs(const ubx_nav_pvt_t *u_blox_data, crsf_telemetry_gps_t *elrs_data)
{
    // 1. Direct register copy for coordinates (No float math, single clock cycle)
    elrs_data->latitude  = u_blox_data->lat;
    elrs_data->longitude = u_blox_data->lon;

    // 2. Fast integer conversions for other fields
    // Convert speed: u-blox (mm/s) to ELRS (km/h * 10) -> equivalent to dividing by 27.777
    elrs_data->ground_speed = (uint16_t)((u_blox_data->gSpeed * 36) / 1000);

    // Convert heading: u-blox (Degrees * 10^-5) to ELRS (Degrees * 100)
    elrs_data->heading = (uint16_t)(u_blox_data->heading / 1000);

    // Convert altitude: u-blox (mm) to ELRS (meters + 1000m offset)
    elrs_data->altitude = (uint16_t)((u_blox_data->hMSL / 1000) + 1000);

    // Copy satellite count
    elrs_data->satellites = u_blox_data->numSV;
}

// Function to send the complete CRSF packet via UART
void crsf_send_gps_packet(const crsf_telemetry_gps_t *gps_payload)
{
    uint8_t tx_buffer[sizeof(crsf_header_t) + sizeof(crsf_telemetry_gps_t) + 1];
    
    // 1. Prepare Header
    crsf_header_t *header = (crsf_header_t *)tx_buffer;
    header->device_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    header->length = CRSF_GPS_PAYLOAD_SIZE + 2; // Type (1) + Payload + CRC (1)
    header->frame_type = CRSF_FRAMETYPE_GPS;
    
    // 2. Copy Payload (Warning: CRSF requires Big-Endian, u-blox provides Little-Endian)
    // We reverse bytes for standard integer fields
    crsf_telemetry_gps_t *target_payload = (crsf_telemetry_gps_t *)(tx_buffer + sizeof(crsf_header_t));
    target_payload->latitude     = __builtin_bswap32(gps_payload->latitude);
    target_payload->longitude    = __builtin_bswap32(gps_payload->longitude);
    target_payload->ground_speed = __builtin_bswap16(gps_payload->ground_speed);
    target_payload->heading      = __builtin_bswap16(gps_payload->heading);
    target_payload->altitude     = __builtin_bswap16(gps_payload->altitude);
    target_payload->satellites   = gps_payload->satellites;
    
    // 3. Compute Checksum (from frame_type to end of payload)
    uint8_t crc_start_idx = offsetof(crsf_header_t, frame_type);
    uint8_t crc_length = sizeof(crsf_telemetry_gps_t) + 1; // include type byte
    
    uint8_t crc = calculate_crc8_crsf(&tx_buffer[crc_start_idx], crc_length);
    tx_buffer[sizeof(tx_buffer) - 1] = crc;
    
    // 4. Write data to UART
    uart_write_bytes(CRSF_UART_PORT, (const char *)tx_buffer, sizeof(tx_buffer));
}

void crsf_send_battery_packet(uint16_t voltage_v_times_10, uint16_t current_a_times_10, uint32_t fuel_mah, uint8_t percent)
{
    // Total size = Header (3 bytes) + Payload (8 bytes) + CRC (1 byte) = 12 bytes
    uint8_t tx_buffer[3 + 8 + 1];
    
    // 1. Setup CRSF Header
    crsf_header_t *header = (crsf_header_t *)tx_buffer;
    header->device_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    header->length = 1 + 8 + 1; // Type (1) + Payload (8) + CRC (1)
    header->frame_type = CRSF_FRAMETYPE_BATTERY_SENSOR;
    
    // 2. Prepare Payload with Big-Endian conversion
    uint8_t *payload_ptr = tx_buffer + 3;
    
    // Convert Voltage and Current to Big-Endian (16-bit)
    uint16_t be_voltage = __builtin_bswap16(voltage_v_times_10);
    uint16_t be_current = __builtin_bswap16(current_a_times_10);
    
    memcpy(payload_ptr, &be_voltage, 2);
    memcpy(payload_ptr + 2, &be_current, 2);
    
    // Convert 32-bit mAh to 24-bit Big-Endian (3 bytes)
    payload_ptr[4] = (uint8_t)((fuel_mah >> 16) & 0xFF);
    payload_ptr[5] = (uint8_t)((fuel_mah >> 8) & 0xFF);
    payload_ptr[6] = (uint8_t)(fuel_mah & 0xFF);
    
    // Copy remaining capacity percentage (1 byte, no endianness swap needed)
    payload_ptr[7] = percent;
    
    // 3. Compute Checksum (From frame_type to end of payload = 1 + 8 = 9 bytes)
    uint8_t crc = calculate_crc8_crsf(&tx_buffer[2], 9);
    tx_buffer[11] = crc;
    
    // 4. Send packet over UART
    uart_write_bytes(CRSF_UART_PORT, (const char *)tx_buffer, sizeof(tx_buffer));
}

void crsf_send_temp(int16_t temp_celsius)
{
    uint8_t frame[7];

    frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xC8
    frame[1] = 5;                              // Payload 
    frame[2] = CRSF_FRAMETYPE_TEMP;            // 0x0D

    frame[3] = 0; // temp ID
    frame[4] = (uint8_t)((temp_celsius >> 8) & 0xFF);
    frame[5] = (uint8_t)(temp_celsius & 0xFF);

    // CRC calculé sur Type + Payload (du byte 2 au byte 6 inclus = 5 octets)
    frame[6] = calculate_crc8_crsf(&frame[2], 4);

    // Envoi sur l'UART de télémétrie
    uart_write_bytes(CRSF_UART_PORT, (const char *)frame, sizeof(frame));
}

void crsf_send_rpm(uint16_t rpm)
{
    uint8_t frame[8];

    frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xC8
    frame[1] = 6;                              // Payload
    frame[2] = CRSF_FRAMETYPE_RPM;             // 0x0C

// Encodage 24-bit Big-Endian (MSB en premier)
    frame[3] = 0;
    frame[4] = (uint8_t)((rpm >> 16) & 0xFF);
    frame[5] = (uint8_t)((rpm >> 8) & 0xFF);
    frame[6] = (uint8_t)(rpm & 0xFF);

    // CRC calculé sur Type + Payload (du byte 2 au byte 6 inclus = 5 octets)
    frame[7] = calculate_crc8_crsf(&frame[2], 5);

    // Envoi UART
    uart_write_bytes(CRSF_UART_PORT, (const char *)frame, sizeof(frame));
}

void crsf_send_attitude(int16_t pitch_deg, int16_t roll_deg, int16_t yaw_deg)
{
    uint8_t frame[10];

    frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xC8
    frame[1] = 8;                              // Type (1) + Payload (6) + CRC (1) = 8
    frame[2] = CRSF_FRAMETYPE_ATTITUDE;        // 0x1E

    // Pitch (Big-Endian)
    frame[3] = (uint8_t)((pitch_deg >> 8) & 0xFF);
    frame[4] = (uint8_t)(pitch_deg & 0xFF);

    // Roll (Big-Endian)
    frame[5] = (uint8_t)((roll_deg >> 8) & 0xFF);
    frame[6] = (uint8_t)(roll_deg & 0xFF);

    // Yaw (Big-Endian)
    frame[7] = (uint8_t)((yaw_deg >> 8) & 0xFF);
    frame[8] = (uint8_t)(yaw_deg & 0xFF);

    // CRC calculé du type jusqu'au dernier octet du Yaw (indices 2 à 8)
    frame[9] = calculate_crc8_crsf(&frame[2], 7);

    // Envoi sur l'UART
    uart_write_bytes(CRSF_UART_PORT, (const char *)frame, sizeof(frame));
}