#ifndef UTILS
#define UTILS

#include <sam3s4b.h>

struct sSensorData
{
	sSensorData(){ pkt[0]='p'; pkt[1]='k'; pkt[2]='t'; }
	char pkt[3];
	int16_t mAccX;
	int16_t mAccY;
	int16_t mAccZ;

	float mGyroX;
	float mGyroY;
	float mGyroZ;
	
	int16_t mMagX;
	int16_t mMagY;
	int16_t mMagZ;

	int16_t mTemperature;
	
	float mQ0;
	float mQ1;
	float mQ2;
	float mQ3;
	

};

#define deg2rad(x) x*0.0174532925

namespace jani {
	template<class T>
	T abs(T v) { if(v<0) return -v; else return v; }
	int strlen(const char *data);
	bool checkAndSet(volatile bool &s);
	uint32_t crc32c(uint32_t crc,  uint8_t *buf, int len);
};

#endif
