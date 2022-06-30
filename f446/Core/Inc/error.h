/*
 * error.h
 *
 *  Created on: Feb 27, 2022
 *      Author: lzr93
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_
#define TRUE 1
#define FALSE 0
typedef struct
 {
    I2C_HandleTypeDef* instance;
    uint16_t sdaPin;
    GPIO_TypeDef* sdaPort;
    uint16_t sclPin;
    GPIO_TypeDef* sclPort;
} I2C_Module_t;

void I2C_ClearBusyFlagErratum(I2C_Module_t* i2c, uint32_t timeout);


#endif /* INC_ERROR_H_ */
