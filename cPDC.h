#ifndef __CPDC_H__
#define __CPDC_H__

#include <sam3s4b.h>

typedef struct pdc_packet {
	uint32_t ul_addr;
	uint32_t ul_size;
} pdc_packet_t;

class cPDC
{
public:
	cPDC(Pdc *pdc);
	~cPDC();
	void enableTransfer(uint32_t controls);
	void txInit(pdc_packet_t *packet, pdc_packet_t *next_packet=0);
	void sendUART(const char *data, int len=-1);
	uint32_t count();
private:
	Pdc *mPDC;
};

#endif
