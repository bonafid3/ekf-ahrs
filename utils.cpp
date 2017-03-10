#include "utils.h"

namespace jani {
	int strlen(const char *data)
	{
		int i=0;
		while(data[i] != '\0') { i++; }
		return i;
	}

	bool checkAndSet(volatile bool &s)
	{
		bool res = s;
		if(s) { s = false; }
		return res;
	}
	
	#define POLY 0xedb88320
	
	uint32_t crc32c(uint32_t crc, uint8_t *buf, int len)
	{
		int k;

		crc = ~crc;
		while (len--) {
			crc ^= *buf++;
			for (k = 0; k < 8; k++) {
				crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
			}
		}
		return ~crc;
	}	
	
};
