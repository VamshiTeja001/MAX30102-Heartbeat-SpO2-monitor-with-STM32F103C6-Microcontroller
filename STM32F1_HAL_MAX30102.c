#include "stm32f1_hal_max30102.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

void max30102_init(max30102_t *mcf, I2C_HandleTypeDef *hi2c)
{
    mcf->_ui2c = hi2c;
    mcf->_interrupt_flag = 0;
    memset(mcf->_ir_samples, 0, AX30102_SAMPLE_BITS_LEN * sizeof(uint32_t));
    memset(mcf->_red_samples, 0, AX30102_SAMPLE_BITS_LEN * sizeof(uint32_t));
}

void max30102_write(max30102_t *mcf, uint8_t reg, uint8_t *buf, uint16_t buflen)
{

    uint8_t *payload = (uint8_t *)malloc((buflen+1)* sizeof(uint8_t));
    *payload = reg;
    if(buf !=NULL &&buflen !=0)
        memcpy(payload, buf, buflen);
    HAL_I2CMaster_Transmit(mcf->_ui2c,MAX30102_I2C_SL_ADDR << 1, payload, buflen + 1, MAX30102_I2C_TIMEOUT);
    free(payload);

}

void max30102_read(max30102_t *mcf, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t reg_addr = reg;
    HAL_I2C_Master_Transmit(mcf->_ui2c, MAX30102_I2C_SL_ADDR << 1, &reg_addr, 1, MAX30102_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(mcf->_ui2c, MAX30102_I2C_SL_ADDR << 1, buf, buflen, MAX30102_I2C_TIMEOUT);
}

void max30102_reset(max30102_t *mcf)
{
    uint8_t val = 0x40;
    max30102_write(mcf, MAX30102_MODE_CONFIGURATION_ADDR, &val, 1);
}

void max30102_on_interrupt(max30102_t *mcf)
{
    mcf->_interrupt_flag = 1;
}


uint8_t max30102_has_interrupt(max30102_t *mcf)
{
    return mcf->_interrupt_flag;
}

void max30102_set_a_full(max30102_t *mcf, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_A_FULL);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_A_FULL);
    max30102_write(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
}


void max30102_set_ppg_rdy(max30102_t *mcf, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_PPG_RDY);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_PPG_RDY);
    max30102_write(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
}


void max30102_set_alc_ovf(max30102_t *mcf, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_ALC_OVF);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_ALC_OVF);
    max30102_write(mcf, MAX30102_INTERRUPT_ENABLE_1_ADDR, &reg, 1);
}


void max30102_set_die_temp_rdy(max30102_t *mcf, uint8_t enable)
{
    uint8_t reg = (enable & 0x01) << MAX30102_INTERRUPT_DIE_TEMP_RDY;
    max30102_write(mcf, MAX30102_INTERRUPT_ENABLE_2_ADDR, &reg, 1);
}


void max30102_set_die_temp_en(max30102_t *mcf, uint8_t enable)
{
    uint8_t reg = (enable & 0x01) << MAX30102_TEMPERATURE_DATA_DIE_TEMP_INT_TEMP_EN;
    max30102_write(mcf, MAX30102_TEMPERATURE_DATA_DIE_TEMP_CONFIGURATION, &reg, 1);
}


// void max30102_on_interrupt(max30102_t *mcf)
// {
//     mcf->_interrupt_flag = 1;
// }


// uint8_t max30102_has_interrupt(max30102_t *mcf)
// {
//     return mcf->_interrupt_flag;
// }


void max30102_interrupt_handler(max30102_t *mcf)
{
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30102_read(mcf, MAX30102_INTERRUPT_STATUS_1_ADDR, reg, 2);

    if ((reg[0] >> MAX30102_INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
        max30102_read_fifo(mcf);
    }

    if ((reg[0] >> MAX30102_INTERRUPT_PPG_RDY) & 0x01)
    {
        // New FIFO data ready
    }

    if ((reg[0] >> MAX30102_INTERRUPT_ALC_OVF) & 0x01)
    {
        // Ambient light overflow
    }

    if ((reg[1] >> MAX30102_INTERRUPT_DIE_TEMP_RDY) & 0x01)
    {
        // Temperature data ready
        int8_t temp_int;
        uint8_t temp_frac;
        max30102_read_temp(mcf, &temp_int, &temp_frac);
        // float temp = temp_int + 0.0625f * temp_frac;
    }

    // Reset interrupt flag
    mcf->_interrupt_flag = 0;
}


void max30102_shutdown(max30102_t *mcf, uint8_t shdn)
{
    uint8_t config;
    max30102_read(mcf, MAX30102_MODE_CONFIGURATION_ADDR, &config, 1);
    config = (config & 0x7f) | (shdn << MAX30102_MODE_CONFIGURATION_SHDN);
    max30102_write(mcf, MAX30102_MODE_CONFIGURATION_ADDR, &config, 1);
}


void max30102_set_mode(max30102_t *mcf, max30102_mode_t mode)
{
    uint8_t config;
    max30102_read(mcf, MAX30102_MODE_CONFIGURATION_ADDR, &config, 1);
    config = (config & 0xf8) | mode;
    max30102_write(mcf, MAX30102_MODE_CONFIGURATION_ADDR, &config, 1);
    max30102_clear_fifo(mcf);
}


void max30102_set_sampling_rate(max30102_t *mcf, max30102_sr_t sr)
{
    uint8_t config;
    max30102_read(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
    config = (config & 0x63) | (sr << MAX30102_SPO2_CONFIGURATION_SPO2_SR);
    max30102_write(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
}


void max30102_set_led_pulse_width(max30102_t *mcf, max30102_led_pw_t pw)
{
    uint8_t config;
    max30102_read(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
    config = (config & 0x7c) | (pw << MAX30102_SPO2_CONFIGURATION_LED_PW );
    max30102_write(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
}


void max30102_set_adc_resolution(max30102_t *mcf, max30102_adc_t adc)
{
    uint8_t config;
    max30102_read(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
    config = (config & 0x1f) | (adc << MAX30102_SPO2_CONFIGURATION_SPO2_ADC_RGE);
    max30102_write(mcf, MAX30102_SPO2_CONFIGURATION_ADDR, &config, 1);
}


void max30102_set_led_current_1(max30102_t *mcf, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write(mcf, MAX30102_LED_1_PULSE_AMPLITUDE_ADDR, &pa, 1);
}


void max30102_set_led_current_2(max30102_t *mcf, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write(mcf, MAX30102_LED_2_PULSE_AMPLITUDE_ADDR, &pa, 1);
}


void max30102_set_multi_led_slot_1_2(max30102_t *mcf, max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2)
{
    uint8_t val = 0;
    val |= ((slot1 << MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT1) | (slot2 << MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT2));
    max30102_write(mcf, MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR, &val, 1);
}


void max30102_set_multi_led_slot_3_4(max30102_t *mcf, max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4)
{
    uint8_t val = 0;
    val |= ((slot3 << MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT3) | (slot4 << MAX30102_MULTI_LED_MODE_CONTROL_A_ADDR_SLOT4));
    max30102_write(mcf, MAX30102_MULTI_LED_MODE_CONTROL_B_ADDR, &val, 1);
}


void max30102_set_fifo_config(max30102_t *mcf, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full)
{
    uint8_t config = 0x00;
    config |= smp_ave << MAX30102_FIFO_CONFIGURATION_SMP_AVE;
    config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIGURATION_ROLLOVER_EN);
    config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIGURATION_FIFO_A_FULL);
    max30102_write(mcf, MAX30102_FIFO_CONFIGURATION_ADDR, &config, 1);
}

void max30102_clear_fifo(max30102_t *mcf)
{
    uint8_t val = 0x00;
    max30102_write(mcf, MAX30102_FIFO_WRITE_POINTER, &val, 3);
    max30102_write(mcf, MAX30102_FIFO_READ_COUNTER, &val, 3);
    max30102_write(mcf, MAX30102_FIFO_OVERFLOW_COUNTER, &val, 3);
}


void max30102_read_fifo(max30102_t *mcf)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wr_ptr = 0, rd_ptr = 0;
    max30102_read(mcf, MAX30102_FIFO_WRITE_POINTER, &wr_ptr, 1);
    max30102_read(mcf, MAX30102_FIFO_READ_COUNTER, &rd_ptr, 1);

    int8_t num_samples;

    num_samples = (int8_t)wr_ptr - (int8_t)rd_ptr;
    if (num_samples < 1)
    {
        num_samples += 32;
    }

    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
    for (int8_t i = 0; i < num_samples; i++)
    {
        uint8_t sample[6];
        max30102_read(mcf, MAX30102_FIFO_DATA_REGISTER, sample, 6);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
        uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;
        mcf->_ir_samples[i] = ir_sample;
        mcf->_red_samples[i] = red_sample;
        max30102_plot(ir_sample, red_sample);
    }
}



void max30102_read_temp(max30102_t *mcf, int8_t *temp_int, uint8_t *temp_frac)
{
    max30102_read(mcf, MAX30102_TEMPERATURE_DATA_DIE_TEMP_INT, (uint8_t *)temp_int, 1);
    max30102_read(mcf, MAX30102_TEMPERATURE_DATA_DIE_TEMP_FRC, temp_frac, 1);
}

#ifdef __cplusplus
}
#endif