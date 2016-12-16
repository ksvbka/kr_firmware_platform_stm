#include "i2c.h"
#include "stm32f0xx.h"

static I2C_TypeDef* I2Cx = NULL;

void i2c_init(uint8_t i2c_module)
{
        /* Enable GPIOB clocks*/
        RCC_APB2PeriphClockCmd ( RCC_AHBPeriph_GPIOB , ENABLE );

        /* Configure I2C clock and GPIO*/
        GPIO_InitTypeDef GPIO_InitStructure ;
        GPIO_StructInit(&GPIO_InitStructure );

        if (i2c_module & I2C_MODULE_1) {
                I2Cx = I2C1;
                /* I2C1 clock enable */
                RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C1 , ENABLE );

                /* I2C1 SDA and SCL configuration */
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; /*Open drain*/
                GPIO_Init (GPIOB , &GPIO_InitStructure );

                GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
                GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);

                /* I2C1 Reset */
                RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , ENABLE );
                RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , DISABLE );
        }

        if (i2c_module & I2C_MODULE_2) {
                I2Cx = I2C2;
                /* I2C2 clock enable */
                RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C2 , ENABLE );
                /* I2C2 SDA and SCL configuration */
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 ;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; /*Open drain*/
                GPIO_Init (GPIOB , &GPIO_InitStructure );

                GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
                GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);

                /* I2C2 Reset */
                RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C2 , ENABLE );
                RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C2 , DISABLE );
        }

        /* Configure I2Cx*/
        I2C_InitTypeDef I2C_InitStructure;

        I2C_StructInit(&I2C_InitStructure);
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        I2C_InitStructure.I2C_DigitalFilter = 0;
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_OwnAddress1 = 0xAB;
        I2C_InitStructure.I2C_Timing = 0x10805E89;  //0x50330309; // 400kHz at 48Mhz //0x1045061D
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

        I2C_Init(I2Cx , &I2C_InitStructure );
        I2C_Cmd(I2Cx , ENABLE );
}

uint8_t i2c_read_byte (uint8_t device_addr, uint8_t reg_addr)
{
        int8_t data = 0;

        /* Wait until I2C peripheral isn't busy!*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        /* Wait until the transmit interrupt status is set*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

        /* Send the address of the register you wish to read*/
        I2C_SendData(I2Cx, (uint8_t)reg_addr);

        /* Wait until transfer is complete!*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TC) == RESET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

        data = I2C_ReceiveData(I2Cx);

        /* Wait for the stop condition to be sent*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

        /* Clear the stop flag for next transfers*/
        I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);

        return data;
}

uint8_t i2c_read_data (uint8_t device_addr, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t length)
{
        /* Wait until I2C peripheral isn't busy!*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        /* Wait until the transmit interrupt status is set*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

        /* Send the address of the register you wish to read*/
        I2C_SendData(I2Cx, (uint8_t)reg_addr);

        /* Wait until transfer is complete!*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TC) == RESET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, length, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

        int8_t i;
        for (i = 0; i < length; i++) {
                /* Wait until the RX register is full of luscious data!*/
                while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) == RESET);
                rx_buffer[i] = I2C_ReceiveData(I2Cx);
        }

        /* Wait until STOPF flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

        /* Clear STOPF flag */
        I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);

        return i;
}

uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
        /*Wait until I2C isn't busy*/
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);

        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

        /* Wait until TXIS flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

        /* Send Register address */
        I2C_SendData(I2Cx, reg_addr);

        /* Wait until TCR flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TCR) == RESET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

        /* Wait until TXIS flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

        /* Write data to TXDR */
        I2C_SendData(I2Cx, data);

        /* Wait until STOPF flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

        /* Clear STOPF flag */
        I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);

        return 1;
}

uint8_t i2c_write_data(uint8_t device_addr, uint8_t reg_addr, uint8_t * tx_buffer, uint8_t length)
{
        //Wait until I2C isn't busy
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

        /* Wait until TXIS flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

        /* Send Register address */
        I2C_SendData(I2Cx, reg_addr);

        /* Wait until TCR flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TCR) == RESET);

        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
        I2C_TransferHandling(I2Cx, device_addr, length, I2C_AutoEnd_Mode, I2C_No_StartStop);
        int i;
        for (i = 0; i < length; ++i) {
                /* Wait until TXIS flag is set */
                while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);
                /* Write data to TXDR */
                I2C_SendData(I2Cx, tx_buffer[i]);
        }

        /* Wait until STOPF flag is set */
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

        /* Clear STOPF flag */
        I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);

        return i;
}
