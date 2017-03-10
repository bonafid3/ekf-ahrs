#ifndef __CPMC_H__
#define __CPMC_H__

#include <sam3s4b.h>

class cPMC
{
public:
	cPMC();
	~cPMC();

	uint32_t enablePeriphClk(uint32_t pid);
};

#endif
