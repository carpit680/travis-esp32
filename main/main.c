#include "stdio.h"
#include "esp_chatter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"

static const char *TAG = "Wheel Velocities";

void app_main(void)
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit_enc_left = 0;
    uint32_t pcnt_unit_enc_right = 1;

    // Create rotary encoder instance
    rotary_encoder_config_t config_encoder_left = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_enc_left, 14, 27);
    rotary_encoder_config_t config_encoder_right = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_enc_right, 23, 22);

    rotary_encoder_t *encoder_left = NULL;
    rotary_encoder_t *encoder_right = NULL;

    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_left, &encoder_left));
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_right, &encoder_right));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_left->set_glitch_filter(encoder_left, 1));
    ESP_ERROR_CHECK(encoder_right->set_glitch_filter(encoder_right, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder_left->start(encoder_left));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    // Setup rosserial
    rosserial_setup();

    // Report counter value
    int encoder_left_val = 0;
    int encoder_right_val = 0;
    int encoder_left_prev_val = 0;
    int encoder_right_prev_val = 0;
    float vel_left = 0;
    float vel_right = 0;
    while (1)
    {
        encoder_left_val = encoder_left->get_counter_value(encoder_left);
        encoder_right_val = encoder_right->get_counter_value(encoder_right);

        vel_left = 3.1415 * (encoder_left_val -  encoder_left_prev_val) / 30;
        vel_right = 3.1415 * (encoder_right_val - encoder_right_prev_val) / 30;
        
        ESP_LOGI(TAG, "%f, %f", vel_left, vel_right);
        rosserial_publish(vel_left + vel_right);
        vTaskDelay(pdMS_TO_TICKS(100));

        encoder_left_prev_val = encoder_left_val;
        encoder_right_prev_val = encoder_right_val;
    }
}
