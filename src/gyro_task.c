#include "gyro_task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pid.h"
#include "utils.h"

// #define GYRO_1KHZ

#define MPU_ADDR 0x68
#define I2C_SDA_PIN GPIO_NUM_20
#define I2C_SCL_PIN GPIO_NUM_21
#define I2C_INT_PIN GPIO_NUM_1
#define I2C_FREQ_HZ 400000

#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_CONFIG 0x1A // DLPF
#define REG_SMPLRT_DIV 0x19
#define REG_GYRO_XOUT_H 0x43
#define REG_INT_ENABLE 0x38
#define REG_INT_CFG 0x37
#define REG_ACCEL_CONFIG_2 0x1D

#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

#define I2C_TIMEOUT_MS 50

#define GYRO_CUTOFF_FREQ 45.0f
#define GYRO_LOW_CUTOFF_FREQ 15.0f
#define ACCEL_CUTOFF_FREQ 5.0f

#ifdef  GYRO_1KHZ
#define GYRO_DT 0.001f // 1ms
#define MPU_SR_DIV 0
#else
#define GYRO_DT 0.002f // 2ms
#define MPU_SR_DIV 1
#endif

static i2c_master_bus_handle_t i2c_mpu_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_mpu_dev_handle = NULL;
static TaskHandle_t xGyroTaskHandle = NULL;

portMUX_TYPE g_gyro_spinlock = portMUX_INITIALIZER_UNLOCKED;

typedef struct
{
    float rot_x, rot_y, rot_z; // deg/s
    float ax, ay, az;          // m/s^2
    char valid;
} gyro_t;

gyro_data_t g_gyro_data;
int16_t g_gyro_offsets[3] = {0, 0, 0}; // GX, GY, GZ

static const float GYRO_SCALE = 1.0f / 65.5f; // LSB/(deg/s) pour ±500dps, cf datasheet
static const float ACCEL_SCALE_8G = 1.0f / 4096.0f;

extern bool g_invert_accel[3];
extern QueueHandle_t gyro_queue;

// cutoff could be tuned for latency issue (less is induce more lag)
FilterPT1 filterGyroRoll;
FilterPT1 filterGyroPitch;
FilterPT1 filterGyroYaw;

FilterPT1 filterGyroRoll_low;
FilterPT1 filterGyroPitch_low;
FilterPT1 filterGyroYaw_low;

static void mpu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_mpu_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_mpu_bus_handle, &dev_cfg, &i2c_mpu_dev_handle));
}

static inline esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(i2c_mpu_dev_handle, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return err;
}

static void mpu_configure(void)
{
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write_reg(REG_PWR_MGMT_1, 0x80);        // Reset MPU
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write_reg(REG_PWR_MGMT_1, 0x01);        // sort du sleep, clock source = gyro X (plus stable que interne)
    vTaskDelay(pdMS_TO_TICKS(50));

    mpu_write_reg(REG_GYRO_CONFIG, 0x08);       // ±500 deg/s (FS_SEL = 1)
    mpu_write_reg(REG_CONFIG, 0x03);            // DLPF_CFG=3 (Gyro/Accel: ~41Hz, coupe bien avant Nyquist 125Hz)
    mpu_write_reg(REG_SMPLRT_DIV, MPU_SR_DIV);  // Sample rate de sortie = 1kHz / (1+1) = 500Hz ou 1 / (1+0) = 1KHz
    mpu_write_reg(REG_INT_ENABLE, 0x01);        // Enable interrupts
    mpu_write_reg(REG_INT_CFG, 0x10);           // Interrupt on data ready
    mpu_write_reg(REG_ACCEL_CONFIG, 0x10);      // 8g full scale range
    mpu_write_reg(REG_ACCEL_CONFIG_2, 0x03);    // DLPF_CFG=3 (Gyro/Accel: ~41Hz, coupe bien avant Nyquist 125Hz)
}

#define ALPHA (2.0f * M_PI * ACCEL_CUTOFF_FREQ * GYRO_DT)
#define ACCEL_LPF_ALPHA (ALPHA / (ALPHA + 1.0f))

static inline void filter_accelerometer(float ax_raw, float ay_raw, float az_raw,
                                        float *ax_f, float *ay_f, float *az_f)
{
    static float ax_prev = 0.0f;
    static float ay_prev = 0.0f;
    static float az_prev = 1.0f; // 1g initial assumption

    *ax_f = ax_prev + ACCEL_LPF_ALPHA * (ax_raw - ax_prev);
    *ay_f = ay_prev + ACCEL_LPF_ALPHA * (ay_raw - ay_prev);
    *az_f = az_prev + ACCEL_LPF_ALPHA * (az_raw - az_prev);

    ax_prev = *ax_f;
    ay_prev = *ay_f;
    az_prev = *az_f;
}

IRAM_ATTR static esp_err_t mpu_read_gyro(gyro_t *out, const int16_t *offsets)
{
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;

    // Transaction I2C unique : Écriture de l'adresse du registre puis lecture en rafale (burst)
    esp_err_t ret = i2c_master_transmit_receive(
        i2c_mpu_dev_handle, &reg, 1, buffer, sizeof(buffer), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK)
        return ret;

    int16_t gx = (int16_t)(buffer[8] << 8) | buffer[9];
    int16_t gy = (int16_t)(buffer[10] << 8) | buffer[11];
    int16_t gz = (int16_t)(buffer[12] << 8) | buffer[13];

    if (offsets != NULL)
    {
        out->rot_x = (float)(gx - offsets[0]); // Explicit cast for clarity
        out->rot_y = (float)(gy - offsets[1]);
        out->rot_z = (float)(gz - offsets[2]);
    }
    else
    {
        out->rot_x = gx;
        out->rot_y = gy;
        out->rot_z = gz;
    }

    int16_t ax = (int16_t)(buffer[0] << 8) | buffer[1];
    int16_t ay = (int16_t)(buffer[2] << 8) | buffer[3];
    int16_t az = (int16_t)(buffer[4] << 8) | buffer[5];

    out->ax = ax;
    out->ay = ay;
    out->az = az;

    return ESP_OK;
}

void get_gyro_data(gyro_data_t *data)
{
    portENTER_CRITICAL(&g_gyro_spinlock);
    *data = g_gyro_data;
    portEXIT_CRITICAL(&g_gyro_spinlock);
}

static void mpu_calibrate_gyro(int16_t *gyro_offsets)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    gyro_t gyro_data;

    ESP_LOGI("MPU", "Gyro calibration... Do not move the model.");

    while (valid_samples < 500)
    {
        if (mpu_read_gyro(&gyro_data, NULL) == ESP_OK)
        {
            sum_x += (int32_t)gyro_data.rot_x;
            sum_y += (int32_t)gyro_data.rot_y;
            sum_z += (int32_t)gyro_data.rot_z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_offsets[0] = (int16_t)(sum_x / valid_samples);
    gyro_offsets[1] = (int16_t)(sum_y / valid_samples);
    gyro_offsets[2] = (int16_t)(sum_z / valid_samples);

    ESP_LOGI("MPU", "Offsets calculés -> GX: %d, Y: %d, Z: %d\n", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
}

// Routine d'interruption (ISR) déclenchée par le MPU6500
static void IRAM_ATTR mpu_drdy_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xGyroTaskHandle != NULL)
    {
        // Envoie une notification ultra-rapide à la tâche gyro
        vTaskNotifyGiveFromISR(xGyroTaskHandle, &xHigherPriorityTaskWoken);
    }

    // Force le passage immédiat à la tâche gyro si elle est prioritaire
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

static void init_mpu_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Le MPU6500 émet une impulsion HAUTE par défaut
        .intr_type = GPIO_INTR_POSEDGE,       // Front montant
    };
    gpio_config(&io_conf);

    // Installe le service d'interruption GPIO de l'ESP32
    gpio_install_isr_service(0);
    // Attache le handler d'interruption à la broche DRDY
    gpio_isr_handler_add(I2C_INT_PIN, mpu_drdy_isr_handler, NULL);
}

void gyro_calib(void)
{
    mpu_calibrate_gyro(g_gyro_offsets);
    nvs_save_struct("gyro_offsets", g_gyro_offsets, sizeof(g_gyro_offsets));
}

void gyro_control_task(void *pvParameters)
{
    xGyroTaskHandle = xTaskGetCurrentTaskHandle();

    float cleanRollRate = 0.0f, cleanPitchRate = 0.0f, cleanYawRate = 0.0f;
    float cleanRollRate_low = 0.0f, cleanPitchRate_low = 0.0f, cleanYawRate_low = 0.0f;
    float rawAx = 0.0f, rawAy = 0.0f, rawAz = 0.0f;
    float cleanAx = 0.0f, cleanAy = 0.0f, cleanAz = 0.0f;

    gyro_t gyro_data;
    gyro_data.ax = 0.0f;
    gyro_data.ay = 0.0f;
    gyro_data.az = 0.0f;

    initPT1Filter(&filterGyroRoll, GYRO_CUTOFF_FREQ, GYRO_DT);
    initPT1Filter(&filterGyroPitch, GYRO_CUTOFF_FREQ, GYRO_DT);
    initPT1Filter(&filterGyroYaw, GYRO_CUTOFF_FREQ, GYRO_DT);

    initPT1Filter(&filterGyroRoll_low, GYRO_LOW_CUTOFF_FREQ, GYRO_DT);
    initPT1Filter(&filterGyroPitch_low, GYRO_LOW_CUTOFF_FREQ, GYRO_DT);
    initPT1Filter(&filterGyroYaw_low, GYRO_LOW_CUTOFF_FREQ, GYRO_DT);

    if (nvs_load_struct("gyro_offsets", g_gyro_offsets, sizeof(g_gyro_offsets)) != ESP_OK)
    {
        printf("Error loading gyro_offsets\n");
    }

    // Check I2C bus and recover if needed
    check_i2c(I2C_SDA_PIN, I2C_SCL_PIN);
    vTaskDelay(pdMS_TO_TICKS(50));

    mpu_init();
    mpu_configure();
    init_mpu_interrupt();

    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t drop_count = 0;

    uint64_t timer = esp_timer_get_time();

    vTaskDelay(2000);

    while (1)
    {
        // Blocking wait for notification from ISR
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        // counter++;
        // if (esp_timer_get_time() - timer > 1000000) {
        //     ESP_LOGI("MPU", "Gyro loop : %lu", counter);
        //     timer = esp_timer_get_time();
        //     counter = 0;
        // }
        if (ulNotificationValue > 0)
        {
            // DRDY Interrup ! Direct read and process the data
            bool valid = false;
            if (mpu_read_gyro(&gyro_data, g_gyro_offsets) == ESP_OK)
            {
                gyro_data.rot_x *= GYRO_SCALE;
                gyro_data.rot_y *= GYRO_SCALE;
                gyro_data.rot_z *= GYRO_SCALE;

                rawAx = gyro_data.ax * ACCEL_SCALE_8G;
                rawAy = gyro_data.ay * ACCEL_SCALE_8G;
                rawAz = gyro_data.az * ACCEL_SCALE_8G;

                cleanRollRate_low   = applyPT1Filter(&filterGyroRoll_low, gyro_data.rot_x);
                cleanPitchRate_low  = applyPT1Filter(&filterGyroPitch_low, gyro_data.rot_y);
                cleanYawRate_low    = applyPT1Filter(&filterGyroYaw_low, gyro_data.rot_z);

                cleanRollRate   = applyPT1Filter(&filterGyroRoll, gyro_data.rot_x);
                cleanPitchRate  = applyPT1Filter(&filterGyroPitch, gyro_data.rot_y);
                cleanYawRate    = applyPT1Filter(&filterGyroYaw, gyro_data.rot_z);

                filter_accelerometer(rawAx, rawAy, rawAz, &cleanAx, &cleanAy, &cleanAz);

                valid = true;
            }

            portENTER_CRITICAL(&g_gyro_spinlock);
            if (g_invert_accel[0])
            {
                g_gyro_data.rot_x = -cleanRollRate;
                g_gyro_data.rot_x_low = -cleanRollRate_low;
                g_gyro_data.raw_ax = -rawAx;
                g_gyro_data.ax = -cleanAx;
            }
            else
            {
                g_gyro_data.rot_x = cleanRollRate;
                g_gyro_data.rot_x_low = cleanRollRate_low;
                g_gyro_data.raw_ax = rawAx;
                g_gyro_data.ax = cleanAx;
            }

            if (g_invert_accel[1])
            {
                g_gyro_data.rot_y = -cleanPitchRate;
                g_gyro_data.rot_y_low = -cleanPitchRate_low;
                g_gyro_data.raw_ay = -rawAy;
                g_gyro_data.ay = -cleanAy;
            }
            else
            {
                g_gyro_data.rot_y = cleanPitchRate;
                g_gyro_data.rot_y_low = cleanPitchRate_low;
                g_gyro_data.raw_ay = rawAy;
                g_gyro_data.ay = cleanAy;
            }

            if (g_invert_accel[2])
            {
                g_gyro_data.rot_z = -cleanYawRate;
                g_gyro_data.rot_z_low = -cleanYawRate_low;
                g_gyro_data.raw_az = -rawAz;
                g_gyro_data.az = -cleanAz;
            }
            else
            {
                g_gyro_data.rot_z = cleanYawRate;
                g_gyro_data.rot_z_low = cleanYawRate_low;
                g_gyro_data.raw_az = rawAz;
                g_gyro_data.az = cleanAz;
            }
            g_gyro_data.valid = valid;
            portEXIT_CRITICAL(&g_gyro_spinlock);

            if (xQueueSend(gyro_queue, &g_gyro_data, 0) != pdPASS)
            {
                drop_count++;
            } // Send the updated gyro data to the servo queue
        }
        else
        {
            // Timeout : No interruption received, possibly a missed DRDY. Mark data as invalid.
            portENTER_CRITICAL(&g_gyro_spinlock);
            g_gyro_data.valid = false;
            portEXIT_CRITICAL(&g_gyro_spinlock);
        }

        if (drop_count > 0 && (esp_timer_get_time() - timer) > 5000000)
        {
            //ESP_LOGW("MPU", "Dropped %lu gyro data samples due to queue overflow.", drop_count);
            drop_count = 0;
            timer = esp_timer_get_time();
        }
    }
}
