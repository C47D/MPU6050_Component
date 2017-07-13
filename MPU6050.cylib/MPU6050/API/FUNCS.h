#ifndef `$INSTANCE_NAME`_FUNCS_H
#define `$INSTANCE_NAME`_FUNCS_H

#include "cytypes.h"
#include "cyfitter.h"

#include <stdbool.h>

/* Function declarations */
uint8_t devAddr;
uint8_t buffer[22];

void `$INSTANCE_NAME`_init();
void I2C_`$INSTANCE_NAME`_I2CAddress(uint8_t address);

void `$INSTANCE_NAME`_initialize();
bool `$INSTANCE_NAME`_testConnection();

// AUX_VDDIO register
uint8_t `$INSTANCE_NAME`_getAuxVDDIOLevel();
void `$INSTANCE_NAME`_setAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
uint8_t `$INSTANCE_NAME`_getRate();
void `$INSTANCE_NAME`_setRate(uint8_t rate);

// CONFIG register
uint8_t `$INSTANCE_NAME`_getExternalFrameSync();
void `$INSTANCE_NAME`_setExternalFrameSync(uint8_t sync);
uint8_t `$INSTANCE_NAME`_getDLPFMode();
void `$INSTANCE_NAME`_setDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
uint8_t `$INSTANCE_NAME`_getFullScaleGyroRange();
void `$INSTANCE_NAME`_setFullScaleGyroRange(uint8_t range);

// ACCEL_CONFIG register
bool `$INSTANCE_NAME`_getAccelXSelfTest();
void `$INSTANCE_NAME`_setAccelXSelfTest(bool enabled);
bool `$INSTANCE_NAME`_getAccelYSelfTest();
void `$INSTANCE_NAME`_setAccelYSelfTest(bool enabled);
bool `$INSTANCE_NAME`_getAccelZSelfTest();
void `$INSTANCE_NAME`_setAccelZSelfTest(bool enabled);
uint8_t `$INSTANCE_NAME`_getFullScaleAccelRange();
void `$INSTANCE_NAME`_setFullScaleAccelRange(uint8_t range);
uint8_t `$INSTANCE_NAME`_getDHPFMode();
void `$INSTANCE_NAME`_setDHPFMode(uint8_t mode);

// FF_THR register
uint8_t `$INSTANCE_NAME`_getFreefallDetectionThreshold();
void `$INSTANCE_NAME`_setFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
uint8_t `$INSTANCE_NAME`_getFreefallDetectionDuration();
void `$INSTANCE_NAME`_setFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
uint8_t `$INSTANCE_NAME`_getMotionDetectionThreshold();
void `$INSTANCE_NAME`_setMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
uint8_t `$INSTANCE_NAME`_getMotionDetectionDuration();
void `$INSTANCE_NAME`_setMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
uint8_t `$INSTANCE_NAME`_getZeroMotionDetectionThreshold();
void `$INSTANCE_NAME`_setZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
uint8_t `$INSTANCE_NAME`_getZeroMotionDetectionDuration();
void `$INSTANCE_NAME`_setZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
bool `$INSTANCE_NAME`_getTempFIFOEnabled();
void `$INSTANCE_NAME`_setTempFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getXGyroFIFOEnabled();
void `$INSTANCE_NAME`_setXGyroFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getYGyroFIFOEnabled();
void `$INSTANCE_NAME`_setYGyroFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getZGyroFIFOEnabled();
void `$INSTANCE_NAME`_setZGyroFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getAccelFIFOEnabled();
void `$INSTANCE_NAME`_setAccelFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave2FIFOEnabled();
void `$INSTANCE_NAME`_setSlave2FIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave1FIFOEnabled();
void `$INSTANCE_NAME`_setSlave1FIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave0FIFOEnabled();
void `$INSTANCE_NAME`_setSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool `$INSTANCE_NAME`_getMultiMasterEnabled();
void `$INSTANCE_NAME`_setMultiMasterEnabled(bool enabled);
bool `$INSTANCE_NAME`_getWaitForExternalSensorEnabled();
void `$INSTANCE_NAME`_setWaitForExternalSensorEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave3FIFOEnabled();
void `$INSTANCE_NAME`_setSlave3FIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlaveReadWriteTransitionEnabled();
void `$INSTANCE_NAME`_setSlaveReadWriteTransitionEnabled(bool enabled);
uint8_t `$INSTANCE_NAME`_getMasterClockSpeed();
void `$INSTANCE_NAME`_setMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t `$INSTANCE_NAME`_getSlaveAddress(uint8_t num);
void `$INSTANCE_NAME`_setSlaveAddress(uint8_t num, uint8_t address);
uint8_t `$INSTANCE_NAME`_getSlaveRegister(uint8_t num);
void `$INSTANCE_NAME`_setSlaveRegister(uint8_t num, uint8_t reg);
bool `$INSTANCE_NAME`_getSlaveEnabled(uint8_t num);
void `$INSTANCE_NAME`_setSlaveEnabled(uint8_t num, bool enabled);
bool `$INSTANCE_NAME`_getSlaveWordByteSwap(uint8_t num);
void `$INSTANCE_NAME`_setSlaveWordByteSwap(uint8_t num, bool enabled);
bool `$INSTANCE_NAME`_getSlaveWriteMode(uint8_t num);
void `$INSTANCE_NAME`_setSlaveWriteMode(uint8_t num, bool mode);
bool `$INSTANCE_NAME`_getSlaveWordGroupOffset(uint8_t num);
void `$INSTANCE_NAME`_setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t `$INSTANCE_NAME`_getSlaveDataLength(uint8_t num);
void `$INSTANCE_NAME`_setSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t `$INSTANCE_NAME`_getSlave4Address();
void `$INSTANCE_NAME`_setSlave4Address(uint8_t address);
uint8_t `$INSTANCE_NAME`_getSlave4Register();
void `$INSTANCE_NAME`_setSlave4Register(uint8_t reg);
void `$INSTANCE_NAME`_setSlave4OutputByte(uint8_t data);
bool `$INSTANCE_NAME`_getSlave4Enabled();
void `$INSTANCE_NAME`_setSlave4Enabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave4InterruptEnabled();
void `$INSTANCE_NAME`_setSlave4InterruptEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlave4WriteMode();
void `$INSTANCE_NAME`_setSlave4WriteMode(bool mode);
uint8_t `$INSTANCE_NAME`_getSlave4MasterDelay();
void `$INSTANCE_NAME`_setSlave4MasterDelay(uint8_t delay);
uint8_t `$INSTANCE_NAME`_getSlate4InputByte();

// I2C_MST_STATUS register
bool `$INSTANCE_NAME`_getPassthroughStatus();
bool `$INSTANCE_NAME`_getSlave4IsDone();
bool `$INSTANCE_NAME`_getLostArbitration();
bool `$INSTANCE_NAME`_getSlave4Nack();
bool `$INSTANCE_NAME`_getSlave3Nack();
bool `$INSTANCE_NAME`_getSlave2Nack();
bool `$INSTANCE_NAME`_getSlave1Nack();
bool `$INSTANCE_NAME`_getSlave0Nack();

// INT_PIN_CFG register
bool `$INSTANCE_NAME`_getInterruptMode();
void `$INSTANCE_NAME`_setInterruptMode(bool mode);
bool `$INSTANCE_NAME`_getInterruptDrive();
void `$INSTANCE_NAME`_setInterruptDrive(bool drive);
bool `$INSTANCE_NAME`_getInterruptLatch();
void `$INSTANCE_NAME`_setInterruptLatch(bool latch);
bool `$INSTANCE_NAME`_getInterruptLatchClear();
void `$INSTANCE_NAME`_setInterruptLatchClear(bool clear);
bool `$INSTANCE_NAME`_getFSyncInterruptLevel();
void `$INSTANCE_NAME`_setFSyncInterruptLevel(bool level);
bool `$INSTANCE_NAME`_getFSyncInterruptEnabled();
void `$INSTANCE_NAME`_setFSyncInterruptEnabled(bool enabled);
bool `$INSTANCE_NAME`_getI2CBypassEnabled();
void `$INSTANCE_NAME`_setI2CBypassEnabled(bool enabled);
bool `$INSTANCE_NAME`_getClockOutputEnabled();
void `$INSTANCE_NAME`_setClockOutputEnabled(bool enabled);

// INT_ENABLE register
uint8_t `$INSTANCE_NAME`_getIntEnabled();
void `$INSTANCE_NAME`_setIntEnabled(uint8_t enabled);
bool `$INSTANCE_NAME`_getIntFreefallEnabled();
void `$INSTANCE_NAME`_setIntFreefallEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntMotionEnabled();
void `$INSTANCE_NAME`_setIntMotionEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntZeroMotionEnabled();
void `$INSTANCE_NAME`_setIntZeroMotionEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntFIFOBufferOverflowEnabled();
void `$INSTANCE_NAME`_setIntFIFOBufferOverflowEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntI2CMasterEnabled();
void `$INSTANCE_NAME`_setIntI2CMasterEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntDataReadyEnabled();
void `$INSTANCE_NAME`_setIntDataReadyEnabled(bool enabled);

// INT_STATUS register
uint8_t `$INSTANCE_NAME`_getIntStatus();
bool `$INSTANCE_NAME`_getIntFreefallStatus();
bool `$INSTANCE_NAME`_getIntMotionStatus();
bool `$INSTANCE_NAME`_getIntZeroMotionStatus();
bool `$INSTANCE_NAME`_getIntFIFOBufferOverflowStatus();
bool `$INSTANCE_NAME`_getIntI2CMasterStatus();
bool `$INSTANCE_NAME`_getIntDataReadyStatus();

// ACCEL_*OUT_* registers
void `$INSTANCE_NAME`_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
void `$INSTANCE_NAME`_getMotion9t(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz, int16_t* t);
void `$INSTANCE_NAME`_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void `$INSTANCE_NAME`_getMotion6t(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t);
void `$INSTANCE_NAME`_getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t `$INSTANCE_NAME`_getAccelerationX();
int16_t `$INSTANCE_NAME`_getAccelerationY();
int16_t `$INSTANCE_NAME`_getAccelerationZ();

// TEMP_OUT_* registers
int16_t `$INSTANCE_NAME`_getTemperature();

// GYRO_*OUT_* registers
void `$INSTANCE_NAME`_getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t `$INSTANCE_NAME`_getRotationX();
int16_t `$INSTANCE_NAME`_getRotationY();
int16_t `$INSTANCE_NAME`_getRotationZ();

// EXT_SENS_DATA_* registers
uint8_t `$INSTANCE_NAME`_getExternalSensorByte(int position);
uint16_t `$INSTANCE_NAME`_getExternalSensorWord(int position);
uint32_t `$INSTANCE_NAME`_getExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
bool `$INSTANCE_NAME`_getXNegMotionDetected();
bool `$INSTANCE_NAME`_getXPosMotionDetected();
bool `$INSTANCE_NAME`_getYNegMotionDetected();
bool `$INSTANCE_NAME`_getYPosMotionDetected();
bool `$INSTANCE_NAME`_getZNegMotionDetected();
bool `$INSTANCE_NAME`_getZPosMotionDetected();
bool `$INSTANCE_NAME`_getZeroMotionDetected();

// I2C_SLV*_DO register
void `$INSTANCE_NAME`_setSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
bool `$INSTANCE_NAME`_getExternalShadowDelayEnabled();
void `$INSTANCE_NAME`_setExternalShadowDelayEnabled(bool enabled);
bool `$INSTANCE_NAME`_getSlaveDelayEnabled(uint8_t num);
void `$INSTANCE_NAME`_setSlaveDelayEnabled(uint8_t num, bool enabled);

// SIGNAL_PATH_RESET register
void `$INSTANCE_NAME`_resetGyroscopePath();
void `$INSTANCE_NAME`_resetAccelerometerPath();
void `$INSTANCE_NAME`_resetTemperaturePath();

// MOT_DETECT_CTRL register
uint8_t `$INSTANCE_NAME`_getAccelerometerPowerOnDelay();
void `$INSTANCE_NAME`_setAccelerometerPowerOnDelay(uint8_t delay);
uint8_t `$INSTANCE_NAME`_getFreefallDetectionCounterDecrement();
void `$INSTANCE_NAME`_setFreefallDetectionCounterDecrement(uint8_t decrement);
uint8_t `$INSTANCE_NAME`_getMotionDetectionCounterDecrement();
void `$INSTANCE_NAME`_setMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
bool `$INSTANCE_NAME`_getFIFOEnabled();
void `$INSTANCE_NAME`_setFIFOEnabled(bool enabled);
bool `$INSTANCE_NAME`_getI2CMasterModeEnabled();
void `$INSTANCE_NAME`_setI2CMasterModeEnabled(bool enabled);
void `$INSTANCE_NAME`_switchSPIEnabled(bool enabled);
void `$INSTANCE_NAME`_resetFIFO();
void `$INSTANCE_NAME`_resetI2CMaster();
void `$INSTANCE_NAME`_resetSensors();

// PWR_MGMT_1 register
void `$INSTANCE_NAME`_reset();
bool `$INSTANCE_NAME`_getSleepEnabled();
void `$INSTANCE_NAME`_setSleepEnabled(bool enabled);
bool `$INSTANCE_NAME`_getWakeCycleEnabled();
void `$INSTANCE_NAME`_setWakeCycleEnabled(bool enabled);
bool `$INSTANCE_NAME`_getTempSensorEnabled();
void `$INSTANCE_NAME`_setTempSensorEnabled(bool enabled);
uint8_t `$INSTANCE_NAME`_getClockSource();
void `$INSTANCE_NAME`_setClockSource(uint8_t source);

// PWR_MGMT_2 register
uint8_t `$INSTANCE_NAME`_getWakeFrequency();
void `$INSTANCE_NAME`_setWakeFrequency(uint8_t frequency);
bool `$INSTANCE_NAME`_getStandbyXAccelEnabled();
void `$INSTANCE_NAME`_setStandbyXAccelEnabled(bool enabled);
bool `$INSTANCE_NAME`_getStandbyYAccelEnabled();
void `$INSTANCE_NAME`_setStandbyYAccelEnabled(bool enabled);
bool `$INSTANCE_NAME`_getStandbyZAccelEnabled();
void `$INSTANCE_NAME`_setStandbyZAccelEnabled(bool enabled);
bool `$INSTANCE_NAME`_getStandbyXGyroEnabled();
void `$INSTANCE_NAME`_setStandbyXGyroEnabled(bool enabled);
bool `$INSTANCE_NAME`_getStandbyYGyroEnabled();
void `$INSTANCE_NAME`_setStandbyYGyroEnabled(bool enabled);
bool `$INSTANCE_NAME`_getStandbyZGyroEnabled();
void `$INSTANCE_NAME`_setStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
uint16_t `$INSTANCE_NAME`_getFIFOCount();

// FIFO_R_W register
uint8_t `$INSTANCE_NAME`_getFIFOByte();
void `$INSTANCE_NAME`_setFIFOByte(uint8_t data);
void `$INSTANCE_NAME`_getFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t `$INSTANCE_NAME`_getDeviceID();
void `$INSTANCE_NAME`_setDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t `$INSTANCE_NAME`_getOTPBankValid();
void `$INSTANCE_NAME`_setOTPBankValid(bool enabled);
int8_t `$INSTANCE_NAME`_getXGyroOffset();
void `$INSTANCE_NAME`_setXGyroOffset(int8_t offset);

// YG_OFFS_TC register
int8_t `$INSTANCE_NAME`_getYGyroOffset();
void `$INSTANCE_NAME`_setYGyroOffset(int8_t offset);

// ZG_OFFS_TC register
int8_t `$INSTANCE_NAME`_getZGyroOffset();
void `$INSTANCE_NAME`_setZGyroOffset(int8_t offset);

// X_FINE_GAIN register
int8_t `$INSTANCE_NAME`_getXFineGain();
void `$INSTANCE_NAME`_setXFineGain(int8_t gain);

// Y_FINE_GAIN register
int8_t `$INSTANCE_NAME`_getYFineGain();
void `$INSTANCE_NAME`_setYFineGain(int8_t gain);

// Z_FINE_GAIN register
int8_t `$INSTANCE_NAME`_getZFineGain();
void `$INSTANCE_NAME`_setZFineGain(int8_t gain);

// XA_OFFS_* registers
int16_t `$INSTANCE_NAME`_getXAccelOffset();
void `$INSTANCE_NAME`_setXAccelOffset(int16_t offset);

// YA_OFFS_* register
int16_t `$INSTANCE_NAME`_getYAccelOffset();
void `$INSTANCE_NAME`_setYAccelOffset(int16_t offset);

// ZA_OFFS_* register
int16_t `$INSTANCE_NAME`_getZAccelOffset();
void `$INSTANCE_NAME`_setZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
int16_t `$INSTANCE_NAME`_getXGyroOffsetUser();
void `$INSTANCE_NAME`_setXGyroOffsetUser(int16_t offset);

// YG_OFFS_USR* register
int16_t `$INSTANCE_NAME`_getYGyroOffsetUser();
void `$INSTANCE_NAME`_setYGyroOffsetUser(int16_t offset);

// ZG_OFFS_USR* register
int16_t `$INSTANCE_NAME`_getZGyroOffsetUser();
void `$INSTANCE_NAME`_setZGyroOffsetUser(int16_t offset);

// INT_ENABLE register (DMP functions)
bool `$INSTANCE_NAME`_getIntPLLReadyEnabled();
void `$INSTANCE_NAME`_setIntPLLReadyEnabled(bool enabled);
bool `$INSTANCE_NAME`_getIntDMPEnabled();
void `$INSTANCE_NAME`_setIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
bool `$INSTANCE_NAME`_getDMPInt5Status();
bool `$INSTANCE_NAME`_getDMPInt4Status();
bool `$INSTANCE_NAME`_getDMPInt3Status();
bool `$INSTANCE_NAME`_getDMPInt2Status();
bool `$INSTANCE_NAME`_getDMPInt1Status();
bool `$INSTANCE_NAME`_getDMPInt0Status();

// INT_STATUS register (DMP functions)
bool `$INSTANCE_NAME`_getIntPLLReadyStatus();
bool `$INSTANCE_NAME`_getIntDMPStatus();

// USER_CTRL register (DMP functions)
bool `$INSTANCE_NAME`_getDMPEnabled();
void `$INSTANCE_NAME`_setDMPEnabled(bool enabled);
void `$INSTANCE_NAME`_resetDMP();

// BANK_SEL register
void `$INSTANCE_NAME`_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR register
void `$INSTANCE_NAME`_setMemoryStartAddress(uint8_t address);

// MEM_R_W register
uint8_t `$INSTANCE_NAME`_readMemoryByte();
void `$INSTANCE_NAME`_writeMemoryByte(uint8_t data);
void `$INSTANCE_NAME`_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
bool `$INSTANCE_NAME`_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);
bool `$INSTANCE_NAME`_writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);

bool `$INSTANCE_NAME`_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem);
bool `$INSTANCE_NAME`_writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
uint8_t `$INSTANCE_NAME`_getDMPConfig1();
void `$INSTANCE_NAME`_setDMPConfig1(uint8_t config);

// DMP_CFG_2 register
uint8_t `$INSTANCE_NAME`_getDMPConfig2();
void `$INSTANCE_NAME`_setDMPConfig2(uint8_t config);

//Magnetometer initialization
void `$INSTANCE_NAME`_setup_compass();

// special methods for MotionApps 2.0 implementation
#ifdef `$INSTANCE_NAME`_INCLUDE_DMP_MOTIONAPPS20
	uint8_t *dmpPacketBuffer;
	uint16_t dmpPacketSize;

	uint8_t `$INSTANCE_NAME`_dmpInitialize();
	bool `$INSTANCE_NAME`_dmpPacketAvailable();

	uint8_t `$INSTANCE_NAME`_dmpSetFIFORate(uint8_t fifoRate);
	uint8_t `$INSTANCE_NAME`_dmpGetFIFORate();
	uint8_t `$INSTANCE_NAME`_dmpGetSampleStepSizeMS();
	uint8_t `$INSTANCE_NAME`_dmpGetSampleFrequency();
	int32_t `$INSTANCE_NAME`_dmpDecodeTemperature(int8_t tempReg);
	
	// Register callbacks after a packet of FIFO data is processed
	//uint8_t `$INSTANCE_NAME`_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
	//uint8_t `$INSTANCE_NAME`_dmpUnregisterFIFORateProcess(inv_obj_func func);
	uint8_t `$INSTANCE_NAME`_dmpRunFIFORateProcesses();
	
	// Setup FIFO for various output
	uint8_t `$INSTANCE_NAME`_dmpSendQuaternion(uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendPacketNumber(uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

	// Get Fixed Point data from FIFO
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpSetLinearAccelFilterCoefficient(float coef);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetControlData(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(VectorFloat *v, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetEIS(int32_t *data, const uint8_t* packet=0);
	
	uint8_t `$INSTANCE_NAME`_dmpGetEuler(float *data, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

	// Get Floating Point data from FIFO
	uint8_t `$INSTANCE_NAME`_dmpGetAccelFloat(float *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

	uint8_t `$INSTANCE_NAME`_dmpProcessFIFOPacket(const unsigned char *dmpData);
	uint8_t `$INSTANCE_NAME`_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

	uint8_t `$INSTANCE_NAME`_dmpSetFIFOProcessedCallback(void (*func) (void));

	uint8_t `$INSTANCE_NAME`_dmpInitFIFOParam();
	uint8_t `$INSTANCE_NAME`_dmpCloseFIFO();
	uint8_t `$INSTANCE_NAME`_dmpSetGyroDataSource(uint8_t source);
	uint8_t `$INSTANCE_NAME`_dmpDecodeQuantizedAccel();
	uint32_t `$INSTANCE_NAME`_dmpGetGyroSumOfSquare();
	uint32_t `$INSTANCE_NAME`_dmpGetAccelSumOfSquare();
	void `$INSTANCE_NAME`_dmpOverrideQuaternion(long *q);
	uint16_t `$INSTANCE_NAME`_dmpGetFIFOPacketSize();
#endif

// special methods for MotionApps 4.1 implementation
#ifdef `$INSTANCE_NAME`_INCLUDE_DMP_MOTIONAPPS41
	uint8_t *dmpPacketBuffer;
	uint16_t dmpPacketSize;

	uint8_t `$INSTANCE_NAME`_dmpInitialize();
	bool `$INSTANCE_NAME`_dmpPacketAvailable();

	uint8_t `$INSTANCE_NAME`_dmpSetFIFORate(uint8_t fifoRate);
	uint8_t `$INSTANCE_NAME`_dmpGetFIFORate();
	uint8_t `$INSTANCE_NAME`_dmpGetSampleStepSizeMS();
	uint8_t `$INSTANCE_NAME`_dmpGetSampleFrequency();
	int32_t `$INSTANCE_NAME`_dmpDecodeTemperature(int8_t tempReg);
	
	// Register callbacks after a packet of FIFO data is processed
	//uint8_t `$INSTANCE_NAME`_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
	//uint8_t `$INSTANCE_NAME`_dmpUnregisterFIFORateProcess(inv_obj_func func);
	uint8_t `$INSTANCE_NAME`_dmpRunFIFORateProcesses();
	
	// Setup FIFO for various output
	uint8_t `$INSTANCE_NAME`_dmpSendQuaternion(uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendPacketNumber(uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t `$INSTANCE_NAME`_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

	// Get Fixed Point data from FIFO
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetMag(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpSetLinearAccelFilterCoefficient(float coef);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetControlData(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetGravity(VectorFloat *v, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetEIS(int32_t *data, const uint8_t* packet=0);
	
	uint8_t `$INSTANCE_NAME`_dmpGetEuler(float *data, Quaternion *q);
	uint8_t `$INSTANCE_NAME`_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

	// Get Floating Point data from FIFO
	uint8_t `$INSTANCE_NAME`_dmpGetAccelFloat(float *data, const uint8_t* packet=0);
	uint8_t `$INSTANCE_NAME`_dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

	uint8_t `$INSTANCE_NAME`_dmpProcessFIFOPacket(const unsigned char *dmpData);
	uint8_t `$INSTANCE_NAME`_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

	uint8_t `$INSTANCE_NAME`_dmpSetFIFOProcessedCallback(void (*func) (void));

	uint8_t `$INSTANCE_NAME`_dmpInitFIFOParam();
	uint8_t `$INSTANCE_NAME`_dmpCloseFIFO();
	uint8_t `$INSTANCE_NAME`_dmpSetGyroDataSource(uint8_t source);
	uint8_t `$INSTANCE_NAME`_dmpDecodeQuantizedAccel();
	uint32_t `$INSTANCE_NAME`_dmpGetGyroSumOfSquare();
	uint32_t `$INSTANCE_NAME`_dmpGetAccelSumOfSquare();
	void `$INSTANCE_NAME`_dmpOverrideQuaternion(long *q);
	uint16_t `$INSTANCE_NAME`_dmpGetFIFOPacketSize();
#endif

#endif

/* [] END OF FILE */
