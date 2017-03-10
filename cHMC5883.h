#ifndef __CHMC5883_H__
#define __CHMC5883_H__

#include "utils.h"
#include "cTWI.h"

#define HMC5833_ADDRESS		0x1E
#define HMC5833_DATA_X_H	0x03
#define HMC5833_DATA_X_L	0x04
#define HMC5833_DATA_Y_H	0x05
#define HMC5833_DATA_Y_L	0x06
#define HMC5833_DATA_Z_H	0x07
#define HMC5833_DATA_Z_L	0x08

#define HMC5833_STATUS		0x09

#define HMC5833_CONFIG_REGA	0x00
#define HMC5833_75HZ		(0b110 << 2)	

#define HMC5833_CONFIG_REGB	0x01
#define HMC5833_2_5GA		(0b011 << 5)

#define HMC5833_MODE_REG	0x02
#define HMC5833_CONTINUOUS	0x00


class cHMC5883
{
public:
	cHMC5883(cTWI &twi);
	~cHMC5883();
	void initialize();
	bool readData(sSensorData &sd);
private:
	cTWI mTWI;
	uint8_t mDevAddr=0;
};

#endif
