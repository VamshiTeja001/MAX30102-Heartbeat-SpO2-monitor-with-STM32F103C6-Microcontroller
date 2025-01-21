#ifndef STM32F1_HAL_MAX30102
#define STM32F1_HAL_MAX30102

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

#define MAX30102_I2C_SL_ADDR 0X57 //PERIPHERAL ADDRESS 0b1010111
#define MAX30102_I2C_TIMEOUT 1000 //1 standard second


///ADC SAMPLE DATA LENGTHS
#define MAX30102_SAMPLE_BYTES_LEN 6
#define MAX30102_SAMPLE_BITS_LEN 32


/// INTERRUPT STATUS REGISTERS 
#define MAX30102_INTERRUPT_STATUS_1_ADDR 0x00
#define MAX30102_INTERRUPT_STATUS_2_ADDR 0x01

#define MAX30102_INTERRUPT_ENABLE_1_ADDR 0X02
#define MAX30102_INTERRUPT_ENABLE_2_ADDR 0X03


#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_PWR_RDY 0
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1


/// FIFO REGISTERS

#define MAX30102_FIFO_WRITE_POINTER 0X04
#define MAX30102_FIFO_OVERFLOW_COUNTER 0X05
#define MAX30102_FIFO_READ_COUNTER 0X06
#define MAX30102_FIFO_DATA_REGISTER 0X07


/// FIFO CONFIGURATION REGISTERS

#define MAX30102_FIFO_CONFIGURATION_ADDR 0X08

#define MAX30102_FIFO_CONFIGURATION_FIFO_A_FULL 0
#define MAX30102_FIFO_CONFIGURATION_ROLLOVER_EN 4
#define MAX30102_FIFO_CONFIGURATION_SMP_AVE 5

/// MODE CONFIGURATION

#define MAX30102_MODE_CONFIGURATION_ADDR 0X09

#define MAX30102_MODE_CONFIGURATION_SHDN 7
#define MAX30102_MODE_CONFIGURATION_RESET 6
#define MAX30102_MODE_CONFIGURATION_MODE 0

/// SPO2 CONFIGURATION 

#define MAX30102_SPO2_CONFIGURATION_ADDR 0X0A

#define MAX30102_SPO2_CONFIGURATION_SPO2_ADC_RGE 5
#define MAX30102_SPO2_CONFIGURATION_SPO2_SR 2
#define MAX30102_SPO2_CONFIGURATION_LED_PW 0

/// LED PULSE AMPLITUDE
#define MAX30102_LED_1_PULSE_AMPLITUDE_ADDR 0X0C
#define MAX30102_LED_2_PULSE_AMPLITUDE_ADDR 0X0D

/// MULTI LED MODE CONTROL REGISTERS

#define MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR 0X11
#define MAX30102_MULTI_LED_MODE_CONTROL_B_ADDR 0X12

#define MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT1 0
#define MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT2 4
#define MAX30102_MULTI_LED_MODE_CONTROL_B_ADDR_SLOT3 0
#define MAX30102_MULTI_LED_MODE_CONTROL_B_ADDR_SLOT4 4

/// TEMPERATURE DATA REGISTER
#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_INT 0X1F
#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_FRC 0X20
#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_CONFIGURATION 0X21

#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_TIN 0
#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_TFRAC 0
#define MAX30102_TEMPERATURE_DATA_DIE_TEMP_INT_TEMP_EN 1  // 1

typedef struct max30102_t
{
    I2C_HandleTypeDef *_ui2c;
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];
    uint8_t _interrupt_flag;
} max30102_t;

typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32,
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

typedef enum max30102_multi_led_ctrl_t
{
    max30102_led_off,
    max30102_led_red,
    max30102_led_ir
} max30102_multi_led_ctrl_t;


void max30102_plot(uint32_t ir_sample, uint32_t red_sample);

void max30102_init(max30102_t *mcf, I2C_HandleTypeDef *hi2c);
void max30102_write(max30102_t *mcf, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30102_read(max30102_t *mcf, uint8_t reg, uint8_t *buf, uint16_t buflen);

void max30102_reset(max30102_t *mcf);

void max30102_set_a_full(max30102_t *mcf, uint8_t enable);
void max30102_set_ppg_rdy(max30102_t *mcf, uint8_t enable);
void max30102_set_alc_ovf(max30102_t *mcf, uint8_t enable);
void max30102_set_die_temp_rdy(max30102_t *mcf, uint8_t enable);
void max30102_set_die_temp_en(max30102_t *mcf, uint8_t enable);

void max30102_on_interrupt(max30102_t *mcf);
uint8_t max30102_has_interrupt(max30102_t *mcf);
void max30102_interrupt_handler(max30102_t *mcf);

void max30102_shutdown(max30102_t *mcf, uint8_t shdn);

void max30102_set_mode(max30102_t *mcf, max30102_mode_t mode);
void max30102_set_sampling_rate(max30102_t *mcf, max30102_sr_t sr);

void max30102_set_led_pulse_width(max30102_t *mcf, max30102_led_pw_t pw);
void max30102_set_adc_resolution(max30102_t *mcf, max30102_adc_t adc);

void max30102_set_led_current_1(max30102_t *mcf, float ma);
void max30102_set_led_current_2(max30102_t *mcf, float ma);
void max30102_set_multi_led_slot_1_2(max30102_t *mcf, max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2);
void max30102_set_multi_led_slot_3_4(max30102_t *mcf, max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4);

void max30102_set_fifo_config(max30102_t *mcf, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);
void max30102_clear_fifo(max30102_t *mcf);
void max30102_read_fifo(max30102_t *mcf);

void max30102_read_temp(max30102_t *mcf, int8_t *temp_int, uint8_t *temp_frac);

#endif