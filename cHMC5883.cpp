#include "cHMC5883.h"

cHMC5883::cHMC5883(cTWI &twi) : mTWI(twi), mDevAddr(HMC5833_ADDRESS)
{
}

cHMC5883::~cHMC5883()
{
}

void cHMC5883::initialize()
{
	bool res = false;

	res = (mTWI.masterWriteByte(mDevAddr, HMC5833_CONFIG_REGA, HMC5833_75HZ)==TWI_SUCCESS);
	res = (mTWI.masterWriteByte(mDevAddr, HMC5833_CONFIG_REGB, HMC5833_2_5GA)==TWI_SUCCESS);
	res = (mTWI.masterWriteByte(mDevAddr, HMC5833_MODE_REG, HMC5833_CONTINUOUS)==TWI_SUCCESS);

	return;
}

bool cHMC5883::readData(sSensorData &sd)
{
	uint8_t status = mTWI.masterReadByte(mDevAddr, HMC5833_DATA_X_H);
	
	twi_packet_t pkt;

	uint8_t buffer[6] = {0};
	pkt.addr[0]=HMC5833_DATA_X_H;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&buffer;
	pkt.chip=mDevAddr;
	pkt.length=6;
	bool res = (mTWI.masterRead(&pkt) == TWI_SUCCESS);

	sd.mMagX = (buffer[0] << 8) | buffer[1];
	sd.mMagY = (buffer[2] << 8) | buffer[3];
	sd.mMagZ = (buffer[4] << 8) | buffer[5];
	
	return res;
}
