#ifndef __CBUFFER_H__
#define __CBUFFER_H__

#include <sam3s4b.h>

// must be power of 2
#define MAX_BUFFER_SIZE 1024

class cBuffer
{
public:
	cBuffer();
	~cBuffer();
	
	void init();
	uint32_t isEmpty();
	uint32_t isFull();
	
	uint32_t append(const uint8_t c);
	uint32_t get(uint8_t &c);
	
	uint16_t size() { return mSize; }
	
private:

	uint16_t mSize=0;
	uint16_t mHead;
	uint16_t mTail;
	uint8_t mBuffer[MAX_BUFFER_SIZE];
};

#endif
