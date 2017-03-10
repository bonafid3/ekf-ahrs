#include "cUART.h"

cUART::cUART(Uart *uart_base) : mUART(uart_base)
{
}

cUART::~cUART()
{
}

uint32_t cUART::init(const sam_uart_opt_t *opts)
{
	uint32_t cd = 0;

	/* Reset and disable receiver & transmitter */
	mUART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX| UART_CR_RXDIS | UART_CR_TXDIS;

	cd = (opts->ul_mck / opts->ul_baudrate) / UART_MCK_DIV;
	if (cd < UART_MCK_DIV_MIN_FACTOR || cd > UART_MCK_DIV_MAX_FACTOR)
		return 1;

	// baud rate generator
	mUART->UART_BRGR = cd;
	
	// Configure mode
	mUART->UART_MR = opts->ul_mode;

	#if (!SAMV71 && !SAMV70 && !SAME70 && !SAMS70)
	// Disable PDC channel
	mUART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	#endif

	// Enable receiver and transmitter 
	mUART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

	return 0;
}

void cUART::enableTX()
{
	mUART->UART_CR = UART_CR_TXEN;
}

void cUART::enableRX()
{
	mUART->UART_CR = UART_CR_RXEN;
}

void cUART::enableInterrupt(uint32_t source)
{
	mUART->UART_IER = source;
}

void cUART::disableInterrupt(uint32_t source)
{
	mUART->UART_IDR = source;
}

uint32_t cUART::status()
{
	return mUART->UART_SR;
}

uint32_t cUART::rxReady()
{
	return (mUART->UART_SR & UART_SR_RXRDY) > 0;
}

uint32_t cUART::txReady()
{
	return (mUART->UART_SR & UART_SR_TXRDY) > 0;
}

uint32_t cUART::read(uint8_t &byte)
{
	if(!rxReady())
		return 0;
	
	byte = mUART->UART_RHR;
	return 1;
}

uint32_t cUART::write(uint8_t byte)
{
	if(!txReady())
		return 0;
	
	mUART->UART_THR = byte;
	return 1;
}
