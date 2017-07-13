#ifndef `$INSTANCE_NAME`_I2C_H
#define `$INSTANCE_NAME`_I2C_H

#include "cytypes.h"
#include "cyfitter.h"

void `$INSTANCE_NAME`_I2CReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* value);
void `$INSTANCE_NAME`_I2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* value);
void `$INSTANCE_NAME`_I2CReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* value);
void `$INSTANCE_NAME`_I2CReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* value);
void `$INSTANCE_NAME`_I2CWriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* value);
void `$INSTANCE_NAME`_I2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t value);
void `$INSTANCE_NAME`_I2CWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value);
void `$INSTANCE_NAME`_I2CWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value);
void `$INSTANCE_NAME`_I2CWriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* value);
void `$INSTANCE_NAME`_I2CWriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t value);

#endif

/* [] END OF FILE */
