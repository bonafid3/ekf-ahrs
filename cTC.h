#ifndef __CTC_H__
#define __CTC_H__

#include <sam3s4b.h>

class cTC
{
public:
	cTC(Tc *tc_base, uint32_t channel);
	~cTC();

	void init(uint32_t mode);
	
	uint32_t status();
	uint32_t counterValue();
	
	uint32_t getRC();
	void writeRA(uint32_t ra);
	void writeRC(uint32_t rc);
	
	void enableInterrupt(uint32_t source);
	
	void start();
	void stop();

private:
	Tc *mTC;
	uint32_t mChn;

};

#endif

