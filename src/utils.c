#include "utils.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NVS_NAMESPACE "storage"

const char* reset_reason_to_str(uint8_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL_PIN";
        case ESP_RST_SW:        return "SOFTWARE_RESET";
        case ESP_RST_PANIC:     return "EXCEPTION_PANIC";
        case ESP_RST_INT_WDT:   return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_EXIT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT_RESET";
        case ESP_RST_SDIO:      return "SDIO_RESET";
        default:                return "UNKNOWN";
    }
}

static const char *TAG_I2C = "I2C_RECOVERY";

void i2c_bus_recovery(int gpio_sda, int gpio_scl) {
    // 1. Configurer SCL et SDA en Open-Drain avec pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_scl) | (1ULL << gpio_sda),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Relâcher les lignes
    gpio_set_level(gpio_scl, 1);
    gpio_set_level(gpio_sda, 1);
    esp_rom_delay_us(10);

    // 2. Générer 9 impulsions d'horloge sur SCL pour forcer le MPU6500 à relâcher SDA
    for (int i = 0; i < 9; i++) {
        gpio_set_level(gpio_scl, 0);
        esp_rom_delay_us(5);
        gpio_set_level(gpio_scl, 1);
        esp_rom_delay_us(5);

        // Si SDA est libéré (remonté à 1), le MPU6500 est sorti de son cycle
        if (gpio_get_level(gpio_sda) == 1) {
            break;
        }
    }

    // 3. Émettre un STOP manuel : SDA passe de 0 à 1 pendant que SCL est à 1
    gpio_set_level(gpio_scl, 0);
    esp_rom_delay_us(5);
    gpio_set_level(gpio_sda, 0);
    esp_rom_delay_us(5);
    gpio_set_level(gpio_scl, 1);
    esp_rom_delay_us(5);
    gpio_set_level(gpio_sda, 1); // Front montant de SDA avec SCL à 1 = STOP
    esp_rom_delay_us(10);

    if (gpio_get_level(gpio_sda) == 0) {
        ESP_LOGE(TAG_I2C, "SDA toujours bloqué à 0 après récupération !");
    } else {
        ESP_LOGI(TAG_I2C, "Bus I2C débloqué avec succès.");
    }
}

void check_i2c(int gpio_sda, int gpio_scl) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_sda),
        .mode = GPIO_MODE_INPUT,           // simple lecture, pas encore open-drain
        .pull_up_en = GPIO_PULLUP_ENABLE,  // au cas où le pull-up externe serait faible/absent
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    esp_rom_delay_us(10); // laisser le pull-up stabiliser la ligne

    int sda_level = gpio_get_level(gpio_sda);
    if (sda_level == 0) {
        i2c_bus_recovery(gpio_sda, gpio_scl);
    }
}

esp_err_t nvs_save_struct(const char *key, const void *data, size_t size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK){
        ESP_LOGE("NVS", "Error opening NVS namespace for save '%s'\n", NVS_NAMESPACE);
    } 

    // Écriture du bloc mémoire brut (BLOB)
    err = nvs_set_blob(handle, key, data, size);
    if (err == ESP_OK) {
        err = nvs_commit(handle); // Validation de l'écriture en Flash
    } else {
        ESP_LOGE("NVS", "Failed to write key %s", key);
    }

    nvs_close(handle);
    return err;
}

// --- CHARGER UNE STRUCTURE DEPUIS LA NVS ---
esp_err_t nvs_load_struct(const char *key, void *data, size_t size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK){
        ESP_LOGE("NVS", "Error opening NVS namespace for load '%s'\n", NVS_NAMESPACE);
    } 

    size_t required_size = size;
    err = nvs_get_blob(handle, key, data, &required_size);
    nvs_close(handle);

    // Vérifie que la clé existe ET que la taille enregistrée correspond à la structure actuelle
    if (err == ESP_OK && required_size != size) {
        ESP_LOGE("NVS", "Failed to load key (length mismatch) %s", key);
        return ESP_ERR_NVS_INVALID_LENGTH;
    } else if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to load key %s", key);
        return err;
    }

    return err;
}