#ifndef __CPIO_H__
#define __CPIO_H__

#include <sam3s4b.h>

class cPIO
{
public:
	cPIO(Pio *pio);
	~cPIO();
	
	void configureRisingEdgeInterrupt(uint32_t mask);
	void configureFallingEdgeInterrupt(uint32_t mask);
	uint32_t interruptMask();
	uint32_t interruptStatus();
	void enableInterrupt(uint32_t source);
	void disableInterrupt(uint32_t source);
	//uint32_t configurePin(uint32_t ul_pin, const uint32_t ul_flags)
	void setPeripheralA(uint32_t mask, uint8_t enablePullUp = false);
	void setPeripheralB(uint32_t mask, uint8_t enablePullUp = false);
	
	void setOutput(uint32_t mask);
	void setInput(uint32_t mask);
	
	void set(uint32_t mask);
	void clear(uint32_t mask);
		
	uint32_t status(uint32_t mask);
private:
	Pio *mPIO;	
};

#endif
