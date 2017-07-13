/* ========================================
 *
 * Copyright Samuel Walsh, 2014
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Samuel Walsh.
 *
 * ========================================
*/

/* Changelog:
*  29/Dec/2015 Carlos Diaz https://github.com/C47D
*  I2C Component must be named I2C, if you use other name you must do
*  changes accordengly to make this file work.
*  Added macros to know what family of PSoC we are using in the project.
*/

#include "`$INSTANCE_NAME`_I2C.h"
#include "`$I2C_INSTANCE`.h"

void `$INSTANCE_NAME`_I2CReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value)
{
	uint8_t i = 0;
#if CY_PSOC4
	`$I2C_INSTANCE`_I2CMasterSendStart(devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	`$I2C_INSTANCE`_I2CMasterSendRestart(devAddr, `$I2C_INSTANCE`_I2C_READ_XFER_MODE);
	while (i++ < (length-1)) {
		*value++ = `$I2C_INSTANCE`_I2CMasterReadByte(`$I2C_INSTANCE`_I2C_ACK_DATA);
	}
	*value = `$I2C_INSTANCE`_I2CMasterReadByte(`$I2C_INSTANCE`_I2C_NAK_DATA);
	`$I2C_INSTANCE`_I2CMasterSendStop();
#elif CY_PSOC5
    `$I2C_INSTANCE`_MasterSendStart(devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	`$I2C_INSTANCE`_MasterSendRestart(devAddr, `$I2C_INSTANCE`_READ_XFER_MODE);
	while (i++ < (length-1)) {
		*value++ = `$I2C_INSTANCE`_MasterReadByte(`$I2C_INSTANCE`_ACK_DATA);
	}
	*value = `$I2C_INSTANCE`_MasterReadByte(`$I2C_INSTANCE`_NAK_DATA);
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_I2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *value)
{
	`$INSTANCE_NAME`_I2CReadBytes(devAddr, regAddr, 1, value);
}

void `$INSTANCE_NAME`_I2CReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value)
{
   	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    `$INSTANCE_NAME`_I2CReadByte(devAddr, regAddr, value);
    *value &= mask;
    *value >>= (bitStart - length + 1);
}

void `$INSTANCE_NAME`_I2CReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *value)
{
	`$INSTANCE_NAME`_I2CReadByte(devAddr, regAddr, value);
	*value = *value & (1 << bitNum);
}
	
void `$INSTANCE_NAME`_I2CWriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value)
{
	uint8_t i = 0;
#if CY_PSOC4
	`$I2C_INSTANCE`_I2CMasterSendStart(devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_I2CMasterWriteByte(*value++);
	}
	`$I2C_INSTANCE`_I2CMasterSendStop();	
#elif CY_PSOC5
    `$I2C_INSTANCE`_MasterSendStart(devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_MasterWriteByte(*value++);
	}
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_I2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t value)
{
	`$INSTANCE_NAME`_I2CWriteBytes(devAddr, regAddr, 1, &value);
}

void `$INSTANCE_NAME`_I2CWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value)
{
	uint8_t b;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	`$INSTANCE_NAME`_I2CReadByte(devAddr, regAddr, &b);
	value <<= (bitStart - length + 1);
	value &= mask;
	b &= ~(mask);
	b |= value;
	`$INSTANCE_NAME`_I2CWriteByte(devAddr, regAddr, b);
}

void `$INSTANCE_NAME`_I2CWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value)
{
	uint8_t b;
	`$INSTANCE_NAME`_I2CReadByte(devAddr, regAddr, &b);
	b = (value != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	`$INSTANCE_NAME`_I2CWriteByte(devAddr, regAddr, b);
}

void `$INSTANCE_NAME`_I2CWriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *value)
{
	uint8_t i = 0;
#if CY_PSOC4
	`$I2C_INSTANCE`_I2CMasterSendStart(devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_I2CMasterWriteByte(((uint8_t)*value) >> 8);
		`$I2C_INSTANCE`_I2CMasterWriteByte((uint8_t)*value++);
	}
	`$I2C_INSTANCE`_I2CMasterSendStop();
#elif CY_PSOC5
    `$I2C_INSTANCE`_MasterSendStart(devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_MasterWriteByte(((uint8_t)*value) >> 8);
		`$I2C_INSTANCE`_MasterWriteByte((uint8_t)*value++);
	}
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_I2CWriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t value)
{
	`$INSTANCE_NAME`_I2CWriteWords(devAddr, regAddr, 1, &value);
}

/* [] END OF FILE */
