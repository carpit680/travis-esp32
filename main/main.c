#include "stdio.h"
#include "esp_chatter.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

static const char *TAG = "Wheel Velocities";
   

///////PIN BINDINGS///////
// Left Motor         ////
// DIR:  D12          ////
// EncA: D14          ////
// PWM:  D13          ////
// EncB: D27          ////
//////////////////////////
// Right Motor        ////
// PWM:  D15          ////
// DIR:  D2           ////
// EncA: D23          ////
// EncB: D22          ////
//////////////////////////



#define GPIO_PWM_LEFT 13  //Set GPIO 13 as PWM0A / Left PWM
#define GPIO_PWM_RIGHT 15   //Set GPIO 15 as PWM0B / Right PWM
#define GPIO_DIR_LEFT GPIO_NUM_12
#define GPIO_DIR_RIGHT GPIO_NUM_2

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_RIGHT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[12], PIN_FUNC_GPIO);
    gpio_set_direction(GPIO_DIR_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_DIR_RIGHT, GPIO_MODE_OUTPUT);
}

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    gpio_set_level(GPIO_DIR_LEFT, 0);
    gpio_set_level(GPIO_DIR_RIGHT, 1);
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    gpio_set_level(GPIO_DIR_LEFT, 1);
    gpio_set_level(GPIO_DIR_RIGHT, 0);
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main(void)
{   
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);

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
