#ifndef __CBYTEARRAY_H__
#define __CBYTEARRAY_H__

#include <sam3s4b.h>
#include "utils.h"
#define MAX_CAPACITY 1024

class cByteArray
{
public:
	cByteArray();
	cByteArray(const char*);
	~cByteArray();
	void init();
	void append(const uint8_t c);
	int find(const char * what, const int offset=0);
	bool extractInts(const int start, int a[], int elems);
	const int size() const;
	void clear();
	const char * c_str();
	char * data(uint32_t offset=0);
	cByteArray & operator=(const cByteArray &rhs);
	cByteArray & operator=(const char *rhs);
	cByteArray & operator+=(const char *rhs);
	
	volatile uint8_t & operator[](int pos);
	volatile const uint8_t & operator[](int pos) const;
	
private:
	uint8_t mMagic[5]={'M','A','G','I','C'};
	volatile uint8_t mData[MAX_CAPACITY];
	int mSize;
};

#endif
