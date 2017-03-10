#ifndef __CMPU6050_H__
#define __CMPU6050_H__

#include "utils.h"
#include "cTWI.h"
#include "MPU_defs.h"

class cHMC5883;

class cMPU6050
{
public:
	cMPU6050(cTWI &twi);
	~cMPU6050();
	bool testConnection();
	
	void reset();
	void setSleepEnabled(bool state);
	void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);
	void setMemoryStartAddress(uint8_t address);
	int16_t getXGyroOffset();
	int16_t getYGyroOffset();
	int16_t getZGyroOffset();
	void initialize(cHMC5883 &hmc);
	
	bool readData(sSensorData &sensorData);
	bool dmpInitialize();
	void setClockOutputEnabled(bool enabled);
	
	int mSamples=0;
	
private:
	cTWI mTWI;
	uint8_t mDevAddr=0;
	
	
	int32_t mSampled[3]={0};
	
};

#endif

