#include "cBuffer.h"

cBuffer::cBuffer()
{
	init();
}

cBuffer::~cBuffer()
{}


void cBuffer::init()
{
	mHead = 0;
	mTail = 0;
	for(int i=0;i<MAX_BUFFER_SIZE;i++)
	{
		mBuffer[i] = 0;
	}
}


uint32_t cBuffer::isFull()
{
	return ((mHead - mTail) == MAX_BUFFER_SIZE) ? 1 : 0;
}

uint32_t cBuffer::isEmpty()
{
	return ((mHead - mTail) == 0U) ? 1 : 0;
}


uint32_t cBuffer::append(const uint8_t c)
{
	if(!isFull())
	{
		mBuffer[ mHead++ & (MAX_BUFFER_SIZE - 1) ] = c;
		mSize++;
		return 1; // ok
	}
	return 0; // error, buffer is full
}

uint32_t cBuffer::get(uint8_t &c)
{
	if(!isEmpty())
	{
		c = mBuffer[ mTail++ & (MAX_BUFFER_SIZE - 1) ];
		mSize--;
		return 1;
	}
	return 0;
}
