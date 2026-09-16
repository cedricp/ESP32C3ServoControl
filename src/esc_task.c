#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "config.h"

#include "esc_task.h"

#define BIT_TIME_US         (1000000 / ESC_BITRATE)         // ~8.68 us
#define HALF_BIT_TIME_US    (BIT_TIME_US / 2)             // ~4.34 us
#define FIRST_SAMPLE_US     (BIT_TIME_US + HALF_BIT_TIME_US) // ~13.02 us (milieu du D0)

extern volatile uint32_t g_esc_temperature;

// Structure d'état de la réception logicielle
typedef struct {
    gpio_num_t          rx_pin;
    gptimer_handle_t    timer;
    QueueHandle_t       rx_queue;
    uint8_t             rx_byte;
    uint8_t             bit_index;
} sw_rx_t;

static sw_rx_t sw_rx_ctx;

inline uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t i;
    crc ^= crc_seed;
    for (i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

inline uint8_t calculate_crc8(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = update_crc8(crc, buf[i]);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// ISR GPTimer : Sample every 8.68 us
// ---------------------------------------------------------------------------
static bool IRAM_ATTR timer_rx_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    sw_rx_t *ctx = (sw_rx_t *)user_ctx;
    BaseType_t high_task_yield = pdFALSE;

    if (ctx->bit_index < 8) {
        int bit = gpio_get_level(ctx->rx_pin);
        if (bit) {
            ctx->rx_byte |= (1 << ctx->bit_index);
        }
        ctx->bit_index++;

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + BIT_TIME_US,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = false,
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    } 
    else {
        // 9th tic = Stop bit (rx finished)
        gptimer_stop(timer);

        xQueueSendFromISR(ctx->rx_queue, &ctx->rx_byte, &high_task_yield);

        // Rearm int for next Start Bit
        gpio_intr_enable(ctx->rx_pin);
    }

    return (high_task_yield == pdTRUE);
}

// ---------------------------------------------------------------------------
// Start bit detection
// ---------------------------------------------------------------------------
static void IRAM_ATTR gpio_rx_isr_handler(void *arg)
{
    sw_rx_t *ctx = (sw_rx_t *)arg;

    // Disable interrupt to avoid noisy signal
    gpio_intr_disable(ctx->rx_pin);

    ctx->rx_byte = 0;
    ctx->bit_index = 0;

    // Program wakeup to read on the middle of the first bit
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = FIRST_SAMPLE_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    
    gptimer_set_raw_count(ctx->timer, 0);
    gptimer_set_alarm_action(ctx->timer, &alarm_config);
    gptimer_start(ctx->timer);
}

// ---------------------------------------------------------------------------
// Rx init
// ---------------------------------------------------------------------------
void sw_serial_rx_init(gpio_num_t gpio_num)
{
    sw_rx_ctx.rx_pin = gpio_num;
    sw_rx_ctx.rx_queue = xQueueCreate(64, sizeof(uint8_t));

    // Configure GPIO with interrupt on falling edge
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    // 2. Initialiser le GPTimer (1 tick = 1 microseconde)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz = résolution 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &sw_rx_ctx.timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_rx_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(sw_rx_ctx.timer, &cbs, &sw_rx_ctx));
    ESP_ERROR_CHECK(gptimer_enable(sw_rx_ctx.timer));

    // 3. Attacher l'ISR GPIO
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_num, gpio_rx_isr_handler, &sw_rx_ctx);
}

bool parse_kiss_frame(const uint8_t *frame, esc_telemetry_t *data) {
    uint8_t computed_crc = calculate_crc8(frame, 9);
    if (computed_crc != frame[9]) {
        return false; // Trame corrompue
    }

    data->temperature    = frame[0];
    data->voltage_mv     = (float)((frame[1] << 8) | frame[2]) * 10;
    data->current_ma     = (float)((frame[3] << 8) | frame[4]) * 10;
    data->mah            = (frame[5] << 8) | frame[6];
    data->erpm           = (uint32_t)((frame[7] << 8) | frame[8]) * 100;

    return true;
}

void esc_telemetry_task(void *pvParameters)
{
    uint8_t byte_in;
    uint8_t frame[10];
    uint8_t frame_idx = 0;
    esc_telemetry_t telemetry_data;

    sw_serial_rx_init(ESC_SWS_RX_GPIO);

    int64_t last_time_rx = 0;

    while (1) {
        // Attend les octets poussés par l'ISR dans la queue
        if (xQueueReceive(sw_rx_ctx.rx_queue, &byte_in, portMAX_DELAY)) {
            
            int64_t now = esp_timer_get_time();

            // Si le silence entre 2 octets dépasse 3000 us (3 ms), c'est un début de trame
            // (À 115200 bauds, le temps entre 2 octets consécutifs est < 1 ms)
            if ((now - last_time_rx) > 3000) {
                frame_idx = 0;
            }
            last_time_rx = now;

            // Enregistrement systématique de l'octet dans le buffer
            if (frame_idx < sizeof(frame)) {
                frame[frame_idx] = byte_in;
                frame_idx++;
            }

            // Quand on a accumulé les 10 octets exacts
            if (frame_idx == 10) {
                if (parse_kiss_frame(frame, &telemetry_data)) {
                    g_esc_temperature = telemetry_data.temperature;
                } else {
                    // CRC incorrect (désynchronisation ou bruit)
                }
                
                // Réinitialisation pour la trame suivante
                frame_idx = 0;
            }
        }
    }
}