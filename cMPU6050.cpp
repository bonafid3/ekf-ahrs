#include "cMPU6050.h"
#include "cHMC5883.h"
#include <math.h>

cMPU6050::cMPU6050(cTWI &twi) : mTWI(twi), mDevAddr(MPU6050_DEFAULT_ADDRESS)
{
	
}

cMPU6050::~cMPU6050()
{
}

void cMPU6050::initialize(cHMC5883 &hmc)
{
	uint8_t data;
	
	data = MPU6050_CLOCK_PLL_XGYRO; // this also disabled sleep mode
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_PWR_MGMT_1, data);

	data = 109; // 8khz/110 72.7 Hz sample rate
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_SMPLRT_DIV, data);

	data = 0x01;
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_INT_ENABLE, data);

	data = (MPU6050_GYRO_FS_2000 << 3);
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_GYRO_CONFIG, data);

	data = (MPU6050_ACCEL_FS_4 << 3);
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_ACCEL_CONFIG, data);
	
	// i2c bypass
	mTWI.masterClearBit(mDevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	mTWI.masterSetBit(mDevAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
	
	hmc.initialize();
	
	mTWI.masterClearBit(mDevAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT);	
	mTWI.masterSetBit(mDevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_I2C_SLV0_ADDR, 0x80 | HMC5833_ADDRESS);
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_I2C_SLV0_REG, 0x03); // address read from
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_I2C_SLV0_CTRL, 0x80 | 0x06); // data length
	
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x01); // data length
}

bool cMPU6050::readData(sSensorData &sd) {
	twi_packet_t pkt;

	uint8_t buffer[20] = {0};
	pkt.addr[0]=MPU6050_RA_ACCEL_XOUT_H;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&buffer;
	pkt.chip=mDevAddr;
	pkt.length=20;
	bool res = (mTWI.masterRead(&pkt) == TWI_SUCCESS);
		
	sd.mAccX = (int16_t)((buffer[0] << 8) | buffer[1]);
	sd.mAccY = (int16_t)((buffer[2] << 8) | buffer[3]);
	sd.mAccZ = (int16_t)((buffer[4] << 8) | buffer[5]);
	
	sd.mAccZ += 1100; // correction!
	
	
	sd.mTemperature = (buffer[6] << 8) | buffer[7];

	int16_t gx = (buffer[8] << 8) | buffer[9];
	int16_t gy = (buffer[10] << 8) | buffer[11];
	int16_t gz = (buffer[12] << 8) | buffer[13];

	if(mSamples < 128) {
		mSampled[0] += gx;
		mSampled[1] += gy;
		mSampled[2] += gz;
		mSamples++;
	} else if(mSamples==128) {
		mSampled[0] /= 128;
		mSampled[1] /= 128;
		mSampled[2] /= 128;
		mSamples++;
	} else {
		gx -= mSampled[0];
		gy -= mSampled[1];
		gz -= mSampled[2];
	}

	// calculate degrees per second but convert to rads / 1/16.4 * (180/PI) 0.06097560975 * 
	sd.mGyroX = deg2rad(gx * GYRO_SENSITIVITY_2000);
	sd.mGyroY = deg2rad(gy * GYRO_SENSITIVITY_2000);
	sd.mGyroZ = deg2rad(gz * GYRO_SENSITIVITY_2000);
	
	sd.mMagX = (buffer[14] << 8) | buffer[15];
	sd.mMagZ = (buffer[16] << 8) | buffer[17];
	sd.mMagY = (buffer[18] << 8) | buffer[19];
	
	sd.mMagX -= -40;
	sd.mMagY -= -60;
	sd.mMagZ -= 32;

	return res;
}

bool cMPU6050::dmpInitialize()
{
	setSleepEnabled(false);
	
	// select bank 16, prefetch, userbank
	setMemoryBank(0x10, true, true);
	
	setMemoryStartAddress(0x06);
	
	// user[16][6] = hw rev
	uint8_t hwRevision = mTWI.masterReadByte(mDevAddr, MPU6050_RA_MEM_R_W);
	
	setMemoryBank(0, false, false);
	
	// check OTP valid
	bool otpValid = (mTWI.masterReadByte(mDevAddr, MPU6050_RA_XG_OFFS_TC) & MPU6050_TC_OTP_BNK_VLD_BIT) == MPU6050_TC_OTP_BNK_VLD_BIT;

	// read gyro offsets
    int8_t xgOffset = getXGyroOffset();
    int8_t ygOffset = getYGyroOffset();
    int8_t zgOffset = getZGyroOffset();

	mTWI.masterReadByte(mDevAddr, MPU6050_RA_USER_CTRL); // ?
	
	twi_packet_t pkt;
	pkt.addr[0]=MPU6050_RA_MEM_R_W;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=(void*)dmpMemoryBank0;
	pkt.chip=mDevAddr;
	pkt.length=MPU6050_MEMORY_BANK_SIZE;
	
	setMemoryBank(0);
	setMemoryStartAddress(0);
	
	if(mTWI.masterWrite(&pkt)!=TWI_SUCCESS) {
		otpValid=false;
	}
	else
	{
		otpValid=true;
	}
	
	setMemoryBank(0);
	setMemoryStartAddress(0);
	
	uint8_t bufa[256]={0};
	pkt.addr[0]=MPU6050_RA_MEM_R_W;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&bufa;
	pkt.chip=mDevAddr;
	pkt.length=MPU6050_MEMORY_BANK_SIZE;
	mTWI.masterRead(&pkt);
	
	for(int i=0; i<256; i++) {
		if(bufa[i] != dmpMemoryBank0[i]) {
			// verify failed
		}
	}
	
	// do other stuff later
	
}

bool cMPU6050::testConnection()
{
	return mTWI.masterReadByte(mDevAddr, MPU6050_RA_WHO_AM_I) == mDevAddr;
}

void cMPU6050::reset()
{
	uint8_t data = (1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_PWR_MGMT_1, data);
}

void cMPU6050::setSleepEnabled(bool state)
{
	uint8_t data = state ? (1<<MPU6050_PWR1_SLEEP_BIT) : ~(1<<MPU6050_PWR1_SLEEP_BIT);
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_PWR_MGMT_1, data);
}

void cMPU6050::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_BANK_SEL, bank);
}

void cMPU6050::setMemoryStartAddress(uint8_t address) {
	mTWI.masterWriteByte(mDevAddr, MPU6050_RA_MEM_START_ADDR, address);
}

int16_t cMPU6050::getXGyroOffset() {
	return mTWI.masterReadWord(mDevAddr, MPU6050_RA_XG_OFFS_USRH);
}

int16_t cMPU6050::getYGyroOffset() {
	return mTWI.masterReadWord(mDevAddr, MPU6050_RA_YG_OFFS_USRH);
}

int16_t cMPU6050::getZGyroOffset() {
	return mTWI.masterReadWord(mDevAddr, MPU6050_RA_ZG_OFFS_USRH);
}

