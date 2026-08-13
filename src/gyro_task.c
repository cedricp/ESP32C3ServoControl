#include "gyro_task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pid.h"


#define MPU_ADDR        0x68
#define I2C_SDA_PIN     GPIO_NUM_20
#define I2C_SCL_PIN     GPIO_NUM_21
#define I2C_INT_PIN     GPIO_NUM_1
#define I2C_FREQ_HZ     400000

#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_CONFIG       0x1A  // DLPF
#define REG_SMPLRT_DIV   0x19
#define REG_GYRO_XOUT_H  0x43
#define REG_INT_ENABLE   0x38
#define REG_INT_CFG      0x37

#define I2C_TIMEOUT_MS   50

#define GYRO_CUTOFF_FREQ 45.0f

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t mpu_handle;

static TaskHandle_t xGyroTaskHandle = NULL;
portMUX_TYPE g_gyro_spinlock = portMUX_INITIALIZER_UNLOCKED;
gyro_data_t g_gyro_data;


static const float GYRO_SCALE = 1.0f / 65.5f;  // LSB/(deg/s) pour ±500dps, cf datasheet


// cutoff coulb be tuned for latency issue (less is induce more lag)
FilterPT1 filterGyroRoll;
FilterPT1 filterGyroPitch;
FilterPT1 filterGyroYaw;

void mpu_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  // pull-ups externes recommandés en vrai montage, mais dépannage OK
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &mpu_handle));
}

static void mpu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_handle, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)));
}

void mpu_configure(void) {
    mpu_write_reg(REG_PWR_MGMT_1, 0x01);      // sort du sleep, clock source = gyro X (plus stable que interne)
    vTaskDelay(pdMS_TO_TICKS(50));

    mpu_write_reg(REG_GYRO_CONFIG, 0x08);     // ±500 deg/s (FS_SEL = 1)
    mpu_write_reg(REG_CONFIG, 0x03);          // DLPF_CFG=3 (Gyro/Accel: ~41Hz, coupe bien avant Nyquist 125Hz)
    mpu_write_reg(REG_SMPLRT_DIV, 0x01);      // Sample rate de sortie = 1kHz / (1+1) = 500Hz
    mpu_write_reg(REG_INT_ENABLE, 0x01);      // Enable interrupts
    mpu_write_reg(REG_INT_CFG, 0x10);         // Interrupt on data ready
}

esp_err_t mpu_read_gyro(gyro_data_t *out, const int16_t *offset) {
    uint8_t reg = REG_GYRO_XOUT_H;
    uint8_t raw[6];

    esp_err_t err = i2c_master_transmit_receive(
        mpu_handle, &reg, 1, raw, 6, pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    if (err != ESP_OK) return err;

    int16_t gx = (raw[0] << 8) | raw[1];
    int16_t gy = (raw[2] << 8) | raw[3];
    int16_t gz = (raw[4] << 8) | raw[5];

    if (offset != NULL) {
        out->x = (gx - offset[0]);
        out->y = (gy - offset[1]);
        out->z = (gz - offset[2]);
    } else {
        out->x = gx;
        out->y = gy;
        out->z = gz;
    }

    return ESP_OK;
}


void get_gyro_data(gyro_data_t *data) {
    portENTER_CRITICAL(&g_gyro_spinlock);
    *data = g_gyro_data;
    portEXIT_CRITICAL(&g_gyro_spinlock);
}

static void mpu_calibrate_gyro(int16_t *gyro_offsets) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    gyro_data_t gyro_data;

    ESP_LOGI("MPU", "Gyro calibration... Do not move the model.");

    while (valid_samples < 500) {
        if (mpu_read_gyro(&gyro_data, NULL) == ESP_OK) {
            sum_x += (int32_t)gyro_data.x;
            sum_y += (int32_t)gyro_data.y;
            sum_z += (int32_t)gyro_data.z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_offsets[0] = (int16_t)(sum_x / valid_samples);
    gyro_offsets[1] = (int16_t)(sum_y / valid_samples);
    gyro_offsets[2] = (int16_t)(sum_z / valid_samples);

    ESP_LOGI("MPU", "Offsets calculés -> X: %d, Y: %d, Z: %d", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
}

// Routine d'interruption (ISR) déclenchée par le MPU6500
static void IRAM_ATTR mpu_drdy_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xGyroTaskHandle != NULL) {
        // Envoie une notification ultra-rapide à la tâche gyro
        vTaskNotifyGiveFromISR(xGyroTaskHandle, &xHigherPriorityTaskWoken);
    }

    // Force le passage immédiat à la tâche gyro si elle est prioritaire
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void init_mpu_interrupt(void) {
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


void gyro_control_task(void *pvParameters) {
    xGyroTaskHandle = xTaskGetCurrentTaskHandle();

    
    // FIX 2 : Enregistrer le handle de la tâche en cours pour l'ISR
    float cleanRollRate  = 0.0f, cleanPitchRate = 0.0f, cleanYawRate = 0.0f;
    int16_t gyro_offsets[3] = {0, 0, 0};
    gyro_data_t gyro_data;
    
    const float dt = 1.0f / 500.0f;
    
    initPT1Filter(&filterGyroRoll,  GYRO_CUTOFF_FREQ, dt); 
    initPT1Filter(&filterGyroPitch, GYRO_CUTOFF_FREQ, dt);
    initPT1Filter(&filterGyroYaw,   GYRO_CUTOFF_FREQ, dt);
    
    mpu_init();
    mpu_configure();
    init_mpu_interrupt();
    
    vTaskDelay(pdMS_TO_TICKS(500));

    mpu_calibrate_gyro(gyro_offsets);
    
    while (1) {
        // Attente bloquante du signal DRDY (timeout de 10ms par sécurité)
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        if (ulNotificationValue > 0) {
            // DRDY Reçu ! Lecture immédiate des registres du MPU
            bool valid = false;
            if (mpu_read_gyro(&gyro_data, gyro_offsets) == ESP_OK) {
                gyro_data.x *= GYRO_SCALE;
                gyro_data.y *= GYRO_SCALE;
                gyro_data.z *= GYRO_SCALE;

                cleanRollRate  = applyPT1Filter(&filterGyroRoll,  gyro_data.x);
                cleanPitchRate = applyPT1Filter(&filterGyroPitch, gyro_data.y);
                cleanYawRate   = applyPT1Filter(&filterGyroYaw,   gyro_data.z);
                valid = true;
            }

            // Mise à jour atomique des globales
            portENTER_CRITICAL(&g_gyro_spinlock);
            g_gyro_data.x = cleanRollRate;
            g_gyro_data.y = cleanPitchRate;
            g_gyro_data.z = cleanYawRate;
            g_gyro_data.valid = valid;
            portEXIT_CRITICAL(&g_gyro_spinlock);

        } else {
            // Timeout : La broche DRDY n'a pas répondu en 10ms (capteur débranché / planté)
            portENTER_CRITICAL(&g_gyro_spinlock);
            g_gyro_data.valid = false;
            portEXIT_CRITICAL(&g_gyro_spinlock);
        }
    }
}
