#ifndef `$INSTANCE_NAME`_LL_I2C_H
#define `$INSTANCE_NAME`_LL_I2C_H

#include "cytypes.h"
#include "cyfitter.h"

extern uint8_t `$INSTANCE_NAME`_devAddr;

void `$INSTANCE_NAME`_ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *value);
void `$INSTANCE_NAME`_ReadByte(uint8_t regAddr, uint8_t *value);
void `$INSTANCE_NAME`_ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value);
void `$INSTANCE_NAME`_ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *value);
void `$INSTANCE_NAME`_WriteBytes(uint8_t regAddr, uint8_t length, uint8_t *value);
void `$INSTANCE_NAME`_WriteByte(uint8_t regAddr, uint8_t value);
void `$INSTANCE_NAME`_WriteBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value);
void `$INSTANCE_NAME`_WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t value);
void `$INSTANCE_NAME`_WriteWords(uint8_t regAddr, uint8_t length, uint16_t *value);
void `$INSTANCE_NAME`_WriteWord(uint8_t regAddr, uint16_t value);

#endif /* `$INSTANCE_NAME`_LL_I2C_H`$INSTANCE_NAME`_ */

/* [] END OF FILE */
