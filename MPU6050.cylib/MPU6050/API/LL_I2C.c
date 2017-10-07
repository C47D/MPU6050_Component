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

/**
 * Changelog:
 * 2017 Carlos Diaz https://github.com/C47D/MPU6050_Component
 * Custom component for the MPU6050.
**/

#include "`$I2C_INSTANCE`.h"
#if defined(CY_SCB_`$I2C_INSTANCE`_H) // I2C based in the SCB block
#include "`$I2C_INSTANCE`_I2C.h"
#endif

#include "`$INSTANCE_NAME`_LL_I2C.h"

uint8_t `$INSTANCE_NAME`_devAddr = `$I2C_ADDRESS`;

void `$INSTANCE_NAME`_ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *value)
{
	uint8_t i = 0;
#if defined(CY_SCB_`$I2C_INSTANCE`_H) // I2C based in the SCB block
	`$I2C_INSTANCE`_I2CMasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	`$I2C_INSTANCE`_I2CMasterSendRestart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_I2C_READ_XFER_MODE);
	while (i++ < (length-1)) {
		*value++ = `$I2C_INSTANCE`_I2CMasterReadByte(`$I2C_INSTANCE`_I2C_ACK_DATA);
	}
	*value = `$I2C_INSTANCE`_I2CMasterReadByte(`$I2C_INSTANCE`_I2C_NAK_DATA);
	`$I2C_INSTANCE`_I2CMasterSendStop();
#else // I2C Based on UDB blocks
    `$I2C_INSTANCE`_MasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	`$I2C_INSTANCE`_MasterSendRestart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_READ_XFER_MODE);
	while (i++ < (length-1)) {
		*value++ = `$I2C_INSTANCE`_MasterReadByte(`$I2C_INSTANCE`_ACK_DATA);
	}
	*value = `$I2C_INSTANCE`_MasterReadByte(`$I2C_INSTANCE`_NAK_DATA);
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_ReadByte(uint8_t regAddr, uint8_t *value)
{
	`$INSTANCE_NAME`_ReadBytes(regAddr, 1, value);
}

void `$INSTANCE_NAME`_ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value)
{
   	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    `$INSTANCE_NAME`_ReadByte(regAddr, value);
    *value &= mask;
    *value >>= (bitStart - length + 1);
}

void `$INSTANCE_NAME`_ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *value)
{
	`$INSTANCE_NAME`_ReadByte(regAddr, value);
	*value = *value & (1 << bitNum);
}
	
void `$INSTANCE_NAME`_WriteBytes(uint8_t regAddr, uint8_t length, uint8_t *value)
{
	uint8_t i = 0;
#if defined(CY_SCB_`$I2C_INSTANCE`_H) // I2C based in the SCB block
	`$I2C_INSTANCE`_I2CMasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_I2CMasterWriteByte(*value++);
	}
	`$I2C_INSTANCE`_I2CMasterSendStop();	
#else
    `$I2C_INSTANCE`_MasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_MasterWriteByte(*value++);
	}
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_WriteByte(uint8_t regAddr, uint8_t value)
{
	`$INSTANCE_NAME`_WriteBytes(regAddr, 1, &value);
}

void `$INSTANCE_NAME`_WriteBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value)
{
	uint8_t b = 0;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    
	`$INSTANCE_NAME`_ReadByte(regAddr, &b);
	value <<= (bitStart - length + 1);
	value &= mask;
	b &= ~(mask);
	b |= value;
	`$INSTANCE_NAME`_WriteByte(regAddr, b);
}

void `$INSTANCE_NAME`_WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t value)
{
	uint8_t b = 0;
    
	`$INSTANCE_NAME`_ReadByte(regAddr, &b);
	b = (value != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	`$INSTANCE_NAME`_WriteByte(regAddr, b);
}

void `$INSTANCE_NAME`_WriteWords(uint8_t regAddr, uint8_t length, uint16_t *value)
{
	uint8_t i = 0;
#if defined(CY_SCB_`$I2C_INSTANCE`_H) // I2C based in the SCB block
	`$I2C_INSTANCE`_I2CMasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_I2C_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_I2CMasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_I2CMasterWriteByte(((uint8_t)*value) >> 8);
		`$I2C_INSTANCE`_I2CMasterWriteByte((uint8_t)*value++);
	}
	`$I2C_INSTANCE`_I2CMasterSendStop();
#else
    `$I2C_INSTANCE`_MasterSendStart(`$INSTANCE_NAME`_devAddr, `$I2C_INSTANCE`_WRITE_XFER_MODE);
	`$I2C_INSTANCE`_MasterWriteByte(regAddr);
	while (i++ < length) {
		`$I2C_INSTANCE`_MasterWriteByte(((uint8_t)*value) >> 8);
		`$I2C_INSTANCE`_MasterWriteByte((uint8_t)*value++);
	}
	`$I2C_INSTANCE`_MasterSendStop();
#endif
}

void `$INSTANCE_NAME`_WriteWord(uint8_t regAddr, uint16_t value)
{
	`$INSTANCE_NAME`_WriteWords(regAddr, 1, &value);
}

/* [] END OF FILE */
