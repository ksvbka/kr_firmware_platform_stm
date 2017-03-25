#include "i2c.h"
#include "stm32f0xx.h"
#include "gpio.h"

static I2C_TypeDef* I2Cx = NULL;

void i2c_init(uint8_t i2c_module)
{
        /* Configure I2C clock and GPIO*/
        if (i2c_module == I2C_MODULE_1) {
                I2Cx = I2C1;
                /* Enable i2c1 clock, reset i2c1*/
                RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
                RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
                RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

               /* i2c1 clock source == sysclk == 48mhz*/
                RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

                /* Setup timing for 400hz fast mode*/
                #define PRESC   15
                #define SCLDEL  1
                #define SDADEL  1
                #define SCLH    14
                #define SCLL    14
                I2Cx->TIMINGR = (SCLL << 0) | (SCLH << 8) | (SDADEL << 16) | (SCLDEL << 20) | (PRESC << 28);

                I2Cx->CR1 |= 1; // enable periph
                /* Configure gpio as i2c function*/
                gpio_init_function(I2C1_SCL_PIN, I2C1_FUNCTION, OPEN_DRAIN, NO_PULL);
                gpio_init_function(I2C1_SDA_PIN, I2C1_FUNCTION, OPEN_DRAIN, NO_PULL);
        }

        if (i2c_module == I2C_MODULE_2) { /* FIX ME !!!*/
                I2Cx = I2C2;
                /* Enable i2c2 clock, reset i2c2*/
                RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
                RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
                RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

               /* i2c1 clock source == sysclk == 48mhz*/
                RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

                /* Setup timing for 400hz fast mode*/
                #define PRESC   15
                #define SCLDEL  1
                #define SDADEL  1
                #define SCLH    14
                #define SCLL    14
                I2Cx->TIMINGR = (SCLL << 0) | (SCLH << 8) | (SDADEL << 16) | (SCLDEL << 20) | (PRESC << 28);

                I2Cx->CR1 |= 1; // enable periph
                // /* Configure gpio as i2c function*/
                gpio_init_function(I2C2_SCL_PIN, I2C2_FUNCTION, OPEN_DRAIN, NO_PULL);
                gpio_init_function(I2C2_SDA_PIN, I2C2_FUNCTION, OPEN_DRAIN, NO_PULL);
        }
}


uint8_t i2c_read_byte (uint8_t device_addr, uint8_t reg_addr)
{
        int8_t data = 0;

        /* Send device address and generate start signal*/
        I2Cx->CR2 = (device_addr << 1) | (1 << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Send reg address want to read, auto increment mode  */
        while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
        I2Cx->TXDR = (reg_addr | (1 << 7));     /* 7 LSBs = reg address, MSB=1 auto-increment*/

        /* Send read command */
        while ((I2Cx->ISR & I2C_ISR_TC) == 0);
        I2Cx->CR2 = (device_addr << 1) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | (1 << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Receive data */
        while ((I2Cx->ISR & I2C_ISR_RXNE) == 0);
        data = I2Cx->RXDR;

        return data;
}

uint8_t i2c_read_data (uint8_t device_addr, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t length)
{
        /* Send device address and generate start signal*/
        I2Cx->CR2 = (device_addr << 1) | (1 << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Send reg address want to read, auto increment mode  */
        while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
        I2Cx->TXDR = (reg_addr | (1 << 7));  /* 7 LSBs = reg address, MSB=1 auto-increment*/

        /* Send read command */
        while ((I2Cx->ISR & I2C_ISR_TC) == 0);
        I2Cx->CR2 = (device_addr << 1) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND| (length << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Read data and store to buffer */
        int i;
        for (i = 0; i < length; ++i) {
                while ((I2Cx->ISR & I2C_ISR_RXNE) == 0);
                rx_buffer[i] = I2Cx->RXDR;
        }
        return i;
}

uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
        /* Send device address and generate start signal*/
        I2Cx->CR2 = (device_addr << 1) | I2C_CR2_AUTOEND | ((1 + 1) << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Send reg address want to write, auto increment mode  */
        while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
        I2Cx->TXDR = reg_addr | (1 << 7);       /* MSB set = auto-increment*/

        /* Write data*/
        while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
        I2Cx->TXDR = data;

        return 1;
}

uint8_t i2c_write_data(uint8_t device_addr, uint8_t reg_addr, uint8_t * tx_buffer, uint8_t length)
{
        /* Send device address and generate start signal*/
        I2Cx->CR2 = (device_addr << 1) | I2C_CR2_AUTOEND | ((length + 1) << 16);
        I2Cx->CR2 |= I2C_CR2_START;

        /* Send start of reg address want to write, auto increment mode  */
        while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
        I2Cx->TXDR = reg_addr | (1 << 7);        /* MSB set = auto-increment*/

        /* Write data*/
        int i;
        for (i = 0; i < length; ++i) {
                while ((I2Cx->ISR & I2C_ISR_TXIS) == 0);
                I2Cx->TXDR = tx_buffer[i];
        }
        return i;
}
