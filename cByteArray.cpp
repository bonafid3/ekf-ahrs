#include "cByteArray.h"
#include "cPIO.h"



cByteArray::cByteArray()
{
	init();
}

 cByteArray::cByteArray(const char*data)
{
	init();
	mSize = 0;
	while(mSize < MAX_CAPACITY && data[mSize] != 0) {
		mData[mSize] = data[mSize];
		mSize++;
	}
}

cByteArray::~cByteArray()
{
}

void cByteArray::init()
{ 
	for(int i=0; i<MAX_CAPACITY; i++) {
		mData[i] = 0;
	}
	mSize = 0;
}

const int cByteArray::size() const
{
	return mSize;
}

void cByteArray::clear()
{
	init();
}

const char * cByteArray::c_str()
{
	mData[mSize] = 0;
	return (char*)mData;
}

char * cByteArray::data(uint32_t offset)
{
	return (char*)mData+offset;
}

cByteArray & cByteArray::operator+=(const char *rhs)
{
	return *this;
}

volatile uint8_t & cByteArray::operator[](int pos)
{
	bool stuff = pos < mSize;
	if(stuff) {
		cPIO pioa(PIOA);
		pioa.set(PIO_PA19);
	}
	return mData[pos];
}

volatile const uint8_t & cByteArray::operator[](int pos) const
{
	bool stuff = pos < mSize;
	if(stuff) {
		cPIO pioa(PIOA);
		pioa.set(PIO_PA19);
	}
	return mData[pos];
}

cByteArray & cByteArray::operator=(const char *rhs)
{
	mSize = 0;
	while(mSize < MAX_CAPACITY && rhs[mSize] != 0) {
		mData[mSize] = rhs[mSize];
		mSize++;
	}
	return *this;
}

void cByteArray::append(const uint8_t c)
{
	if(mSize < MAX_CAPACITY - 1) {
		mData[mSize++] = c;
	}
}

int cByteArray::find(const char * what, const int offset)
{
	int i=offset, j=0, siz=size(), findlen = jani::strlen(what);
	if(siz == 0 || findlen == 0 || siz < offset+1) return -1;
	while(i < siz) {
		j=0;
		while(i < siz && j < findlen && (*this)[i] == what[j]) { 
			if(j == findlen - 1) {
				return i - --findlen;
			}
			i++; j++;
		}
		i++;
	}
	return -1;
}

bool cByteArray::extractInts(const int start, int a[], int elems)
{
	int pos=start, idx=0, siz=size();
	bool numfound = false;
	
	while(pos < siz && idx < elems)
	{
		char c = (*this)[pos];
		if(c > 47 && c < 58) {
			numfound = true; pos++;
			a[idx] = a[idx] * 10 + c - '0';
		} else {
			if(jani::checkAndSet(numfound)) {
				idx++;
			} else {
				pos++;
			}
		}
	}
	
	return idx == elems;
}

