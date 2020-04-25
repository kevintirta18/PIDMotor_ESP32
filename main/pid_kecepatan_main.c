// Library
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"

// PID Macros
#define POS_SAMPLING_PERIOD   100000 // in microseconds
#define MAX_VELOCITY          500     // in rpm

// Motor driver macros
#define PWM_PIN (GPIO_NUM_2)
#define IN1_PIN (GPIO_NUM_16)
#define IN2_PIN (GPIO_NUM_17)

// GPIO Macros
#define PIN_A                 (GPIO_NUM_19)
#define PIN_B                 (GPIO_NUM_18)
#define GPIO_INPUT_PIN_SEL    ((1ULL<<PIN_A) | (1ULL<<PIN_B))
#define ESP_INTR_FLAG_DEFAULT 0

// ADC Macros
#define DEFAULT_VREF    1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64   //Multisampling

// PID global variables
volatile int delta_position = 0;
int power = 0;

// ADC global variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6; //GPIO34
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

// PWM global variables
ledc_timer_config_t ledc_timer = 
{
    .duty_resolution = LEDC_TIMER_13_BIT, // Resolution of PWM duty
    .freq_hz = 500,                       // Frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,    // Low speed timer mode
    .timer_num = LEDC_TIMER_0,            // Timer index
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};

ledc_channel_config_t ledc_channel = 
{
    .channel    = LEDC_CHANNEL_0,
    .duty       = 0,
    .gpio_num   = PWM_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_0
};

uint32_t read_adc() 
{
    uint32_t adc_reading = 0;
    
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t) channel);
    }
    
    adc_reading /= NO_OF_SAMPLES;
        
    return adc_reading;
}

void motor_power(int power, ledc_channel_config_t *ledc_channel){
	uint32_t duty;
    
    if (power < 0) 
    {
        duty = -power;
    }
    else
    {
        duty = power;
    }
        
	if (duty > 8191)
    {
        duty = 8191;
    }
	
    if (power > 0) 
    {   
        // Spin CCW
        gpio_set_level(IN1_PIN, 0); // Turn off
        gpio_set_level(IN2_PIN, 1); // Turn on
        ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, duty);
        ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
	} 
	else if (power < 0) 
    {   
        // Spin CW
        gpio_set_level(IN1_PIN, 1); // Turn on
        gpio_set_level(IN2_PIN, 0); // Turn off
        ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, duty);
        ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel);
	}
	else 
    {
        gpio_set_level(IN1_PIN, 0); // Turn off
        gpio_set_level(IN2_PIN, 0); // Turn off
	}
}

static void IRAM_ATTR pin_a_isr_handler(void* arg)
{
    int pin_a_level, pin_b_level; 
    pin_a_level = gpio_get_level(PIN_A);
    pin_b_level = gpio_get_level(PIN_B);
    if (pin_a_level == pin_b_level) 
    {
        // CCW
        delta_position++;
    }
    else if (pin_a_level != pin_b_level) 
    {
        // CW
        delta_position--;
    }
}

static void IRAM_ATTR pin_b_isr_handler(void* arg)
{
    int pin_a_level, pin_b_level; 
    pin_a_level = gpio_get_level(PIN_A);
    pin_b_level = gpio_get_level(PIN_B);
    if (pin_a_level == pin_b_level) 
    {
        // CW
        delta_position--;
    }
    else if (pin_a_level != pin_b_level) 
    {
        // CCW
        delta_position++;
    }
}

static void pid_callback(void* arg)
{
    float error_integral = 0.0;
    float current_velocity, target, error, control;
    uint32_t adc_reading;
    
    // Read set-point
    adc_reading = read_adc();
    target = ((float) adc_reading / 4095) * MAX_VELOCITY;
    
    // Compute angular velocity in rpm
    current_velocity = ((float) delta_position / 2400) / ((float) POS_SAMPLING_PERIOD / 1000000) * 60;
    
    // Reset delta_position
    delta_position = 0;
    
    // Compute control signal
    error = target - current_velocity;
    error_integral += error;
    control = error * 10 + error_integral * 0.5;
    
    // Set PWM value for motor driver
    power = power + (int) control;
    if (power > 8191) power = 8191;
    
    printf("%.2f,", target);
    printf("%.2f,", error);
    printf("%.2f,", current_velocity);
    printf("%d\n", power);
    
    motor_power(power, &ledc_channel);
}

void app_main(void)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_0);
    
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    
    // Set configuration of timer
    ledc_timer_config(&ledc_timer);

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);
    
    // Set output pins direction
    gpio_set_direction(IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_PIN, GPIO_MODE_OUTPUT);
    
    gpio_config_t io_conf = {
        // Interrupt of rising edge or falling edge
        .intr_type = GPIO_INTR_ANYEDGE,
        // Set as input mode
        .mode = GPIO_MODE_INPUT,
        // Bit mask of the pins that you want to set
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        // Disable pull-down mode
        .pull_down_en = 0,
        // Enable pull-up mode
        .pull_up_en = 1,
    };
    
    // Configure GPIO with the given settings
    gpio_config(&io_conf);

    const esp_timer_create_args_t pid_args = {
        .callback = &pid_callback,
    };

    // Configure timer with the given settings
    esp_timer_handle_t pid;
    esp_timer_create(&pid_args, &pid);

    // Start the timers
    esp_timer_start_periodic(pid, POS_SAMPLING_PERIOD);

    // Install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Hook isr handler for gpio pin A
    gpio_isr_handler_add(PIN_A, pin_a_isr_handler, (void*) PIN_A);
    // Hook isr handler for gpio pin B
    gpio_isr_handler_add(PIN_B, pin_b_isr_handler, (void*) PIN_B);
}
