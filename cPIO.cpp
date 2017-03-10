#include "cPIO.h"

cPIO::cPIO(Pio *pio)
{
	mPIO = pio;
}

cPIO::~cPIO()
{
}

void cPIO::configureRisingEdgeInterrupt(uint32_t mask)
{
	mPIO->PIO_AIMER = mask; // additional interrupt
	mPIO->PIO_REHLSR = mask; // rising edge / high level
	mPIO->PIO_ESR = mask;
}

void cPIO::configureFallingEdgeInterrupt(uint32_t mask)
{
	mPIO->PIO_AIMER = mask; // additional interrupt
	mPIO->PIO_FELLSR = mask; // falling edge / low level status register
	mPIO->PIO_ESR = mask;
	
}

uint32_t cPIO::interruptMask() 
{
	return mPIO->PIO_IMR;
}

uint32_t cPIO::interruptStatus()
{
	return mPIO->PIO_ISR;
}

void cPIO::enableInterrupt(uint32_t source)
{
	mPIO->PIO_IER = source;
}

void cPIO::disableInterrupt(uint32_t source)
{
	mPIO->PIO_IDR = source;
}

void cPIO::setPeripheralA(uint32_t mask, uint8_t enablePullUp)
{
	mPIO->PIO_IDR = mask; // Interrupt disable register

	if (enablePullUp)
		mPIO->PIO_PUER = mask;
	else
		mPIO->PIO_PUDR = mask;

	mPIO->PIO_ABCDSR[0] &= (~mask & PIOA->PIO_ABCDSR[0]);
	mPIO->PIO_ABCDSR[1] &= (~mask & PIOA->PIO_ABCDSR[1]);
	
	mPIO->PIO_PDR = mask; // PIO disable register
}

void cPIO::setPeripheralB(uint32_t mask, uint8_t enablePullUp)
{
	mPIO->PIO_IDR = mask; // Interrupt disable register

	if (enablePullUp)
		mPIO->PIO_PUER = mask;
	else
		mPIO->PIO_PUDR = mask;

	mPIO->PIO_ABCDSR[0] = (mask | PIOA->PIO_ABCDSR[0]);
	mPIO->PIO_ABCDSR[1] &= (~mask & PIOA->PIO_ABCDSR[1]);
	
	mPIO->PIO_PDR = mask; // PIO disable register
}

void cPIO::setOutput(uint32_t mask)
{
	mPIO->PIO_IDR = mask; // Interrupt disable register
	
	mPIO->PIO_OER = mask;
	mPIO->PIO_PER = mask;
}

void cPIO::setInput(uint32_t mask)
{
	mPIO->PIO_IDR = mask; // Interrupt disable register
	mPIO->PIO_PUER = mask; // Pullup enable register
	mPIO->PIO_IFER = mask; // Input filter enable register (deglitch)
	//mPIO->PIO_IFSCER = mask; // Input filter slow clock enable register (debounce) makes the servo jerking
	mPIO->PIO_ODR = mask; // output disable register, therefore input
	mPIO->PIO_PER = mask; // PIO enable register
}


void cPIO::set(uint32_t mask)
{
	mPIO->PIO_SODR = mask;
}

void cPIO::clear(uint32_t mask)
{
	mPIO->PIO_CODR = mask; // Clear Output Data Register
}

uint32_t cPIO::status(uint32_t mask)
{
	return (mPIO->PIO_PDSR & mask)==0?0:1;
}

