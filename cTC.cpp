#include "cTC.h"

cTC::cTC(Tc *tc_base, uint32_t chn) : mTC(tc_base), mChn(chn)
{
}

cTC::~cTC()
{
}

void cTC::init(uint32_t mode)
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	
	channel->TC_CCR = TC_CCR_CLKDIS; // disable clock
	channel->TC_IDR = 0xFFFFFFFF; // disable interrupts
	channel->TC_SR; // clear status register
	channel->TC_CMR = mode;
	
	channel->TC_CCR = TC_CCR_CLKEN;
}

uint32_t cTC::status()
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	return channel->TC_SR;
}

uint32_t cTC::counterValue()
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	return channel->TC_CV;
}


uint32_t cTC::getRC()
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	return channel->TC_RC;
}

void cTC::writeRA(uint32_t ra)
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	channel->TC_RA = ra;
}

void cTC::writeRC(uint32_t rc)
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	channel->TC_RC = rc;
}

void cTC::enableInterrupt(uint32_t source)
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	channel->TC_IER = source;
}

void cTC::start()
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	channel->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void cTC::stop()
{
	TcChannel *channel = mTC->TC_CHANNEL + mChn;
	channel->TC_CCR = TC_CCR_CLKDIS;
}
