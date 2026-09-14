#include "power.h"

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "filter.h"
#include "utils.h"

#include "config.h"

static const char *TAG = "POWER";
adc_cali_handle_t cali0_handle = NULL;
adc_cali_handle_t cali1_handle = NULL;
adc_oneshot_unit_handle_t adc_handle = NULL;

QueueHandle_t power_queue;

#define CALIBRATION_DATA_KEY "calib_data"

enum BatteryType
{
  BATTERY_3S,
  BATTERY_4S,
  BATTERY_UNKNOWN
};

static struct
{
  float voltage_calibration[BATT_NUM];
} calibration_data;

static struct
{
  lpf_u32_t voltage_batt[BATT_NUM];
  int8_t batteryType;
  unsigned long last_time_us;
} battery_data;

#define NUM_ADC_CHANNELS 1

static adc_cali_handle_t cali_handles[NUM_ADC_CHANNELS] = {NULL};

static void init_adc_channel(adc_channel_t channel)
{
  if (channel >= NUM_ADC_CHANNELS)
    return;

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));

  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handles[channel]);
  if (cali_ret == ESP_OK)
  {
    ESP_LOGI(TAG, "Factory ADC channel %d calibration initialized!", channel);
  }
  else
  {
    ESP_LOGE(TAG, "Calibration failed for channel %d (%s). Using raw values.", channel, esp_err_to_name(cali_ret));
  }
}

static int read_adc_channel(adc_channel_t channel)
{
  int raw_reading = 0;
  int voltage_mv = 0;

  if (adc_handle == NULL)
  {
    ESP_LOGE(TAG, "ADC unit %i not initialized", channel);
    return -1;
  }

  esp_err_t ret = adc_oneshot_read(adc_handle, channel, &raw_reading);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "adc_oneshot_read failed on channel %d: %s", channel, esp_err_to_name(ret));
    return -1;
  }

  if (channel < NUM_ADC_CHANNELS && cali_handles[channel] != NULL)
  {
    esp_err_t cali_ret = adc_cali_raw_to_voltage(cali_handles[channel], raw_reading, &voltage_mv);
    if (cali_ret == ESP_OK)
    {
      return voltage_mv;
    }
    ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed: %s", esp_err_to_name(cali_ret));
  }

  return raw_reading;
}

uint16_t  read_filtered_main_battery_mv()
{
  return lpf_u32_value(&battery_data.voltage_batt[BATT_MAIN_ADC_NUM]);
}

uint32_t read_battery_voltage_mv(battery_type_t channel)
{
  float adc = read_adc_channel(channel);
  adc = adc * calibration_data.voltage_calibration[channel];
  return (uint32_t)(adc * 1000.f);
}


uint8_t calcBatteryPercentage()
{
  int32_t voltage_main = read_filtered_main_battery_mv();
  if (battery_data.batteryType == BATTERY_4S)
  {
    // 4S LiPo: 16.8V full, 13.08V empty
    if (voltage_main > 16500)
    {
      return 100;
    }
    else if (voltage_main > 16440)
    {
      return 90;
    }
    else if (voltage_main > 16080)
    {
      return 80;
    }
    else if (voltage_main > 15800)
    {
      return 70;
    }
    else if (voltage_main > 15480)
    {
      return 60;
    }
    else if (voltage_main > 15360)
    {
      return 50;
    }
    else if (voltage_main > 15200)
    {
      return 40;
    }
    else if (voltage_main > 15080)
    {
      return 30;
    }
    else if (voltage_main > 14920)
    {
      return 20;
    }
    else if (voltage_main > 14600)
    {
      return 10;
    }
    else
    {
      return 0;
    }
  }
  else if (battery_data.batteryType == BATTERY_3S)
  {
    // 3S LiPo: 12.6V full, 9.81V empty
    if (voltage_main > 12550)
    {
      return 100;
    }
    else if (voltage_main > 12330)
    {
      return 90;
    }
    else if (voltage_main > 12060)
    {
      return 80;
    }
    else if (voltage_main > 11850)
    {
      return 70;
    }
    else if (voltage_main > 11610)
    {
      return 60;
    }
    else if (voltage_main > 11520)
    {
      return 50;
    }
    else if (voltage_main > 11400)
    {
      return 40;
    }
    else if (voltage_main > 11310)
    {
      return 30;
    }
    else if (voltage_main > 11190)
    {
      return 20;
    }
    else if (voltage_main > 11000)
    {
      return 10;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return 0; // Unknown battery type, cannot estimate percentage
  }
}

void identifyBatteryType()
{
  // Simple heuristic based on voltage_main to determine battery type
  int32_t voltage_main = read_battery_voltage_mv(BATT_MAIN_ADC_NUM);
  if (voltage_main > 14000)
  { // >14V likely a 4S LiPo
    battery_data.batteryType = BATTERY_4S;
  }
  else if (voltage_main > 10800)
  {
    battery_data.batteryType = BATTERY_3S;
  }
  else
  {
    battery_data.batteryType = BATTERY_UNKNOWN;
  }
}

void init_battery_data()
{
  battery_data.batteryType = BATTERY_UNKNOWN;
  battery_data.last_time_us = 0;
  lpf_u32_init(&battery_data.voltage_batt[BATT_MAIN_ADC_NUM], 0, 3);
  identifyBatteryType();
}

void save_current_calibration_data()
{
  if (nvs_save_struct(CALIBRATION_DATA_KEY, &calibration_data, sizeof(calibration_data)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to save calibration data to NVS");
    return;
  }
  ESP_LOGI(TAG, "Calibration data saved to NVS");
}

void load_calibration_data()
{
  if (nvs_load_struct(CALIBRATION_DATA_KEY, &calibration_data, sizeof(calibration_data)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to load calibration data from NVS");
    for (int i = 0; i < BATT_NUM; ++i) calibration_data.voltage_calibration[i] = 1.0f;
    save_current_calibration_data();
  }
  else
  {
    ESP_LOGI(TAG, "Calibration data loaded from NVS");
  }
}

void calibrateVoltmeterSensor(uint8_t channel, float actualValue)
{
  uint32_t rawValue = 0;
  for (int i = 0; i < 10; ++i){
    rawValue += read_adc_channel(channel);
    vTaskDelay(10);
  }
  rawValue /= 10;

  float calibrationFactor = actualValue / (float)rawValue;
  calibration_data.voltage_calibration[channel] = calibrationFactor;
  ESP_LOGI(TAG, "Voltmeter calibration for channel %d: Actual value: %.3f, Calibration factor: %.3f [RAW=%lu]", channel, actualValue, calibrationFactor, rawValue);
  save_current_calibration_data();
}

void update_values()
{
    uint32_t main_batterymv = read_battery_voltage_mv(BATT_MAIN_ADC_NUM);
    lpf_u32_update(&battery_data.voltage_batt[BATT_MAIN_ADC_NUM], main_batterymv);
}

void power_task(void *pvParameters)
{
  batt_stat_t batt_stat;
  // init ADC unit 1
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  load_calibration_data();
  init_battery_data();

  // Main battery
  init_adc_channel(ADC_CHANNEL_0);

  while (1)
  {
    update_values();
    batt_stat.main_batt_voltage_mv  = read_filtered_main_battery_mv();
    batt_stat.main_batt_percent     = calcBatteryPercentage();
    xQueueSend(power_queue, &batt_stat, 0);

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void power_init()
{
  power_queue = xQueueCreate(1, sizeof(batt_stat_t));
}