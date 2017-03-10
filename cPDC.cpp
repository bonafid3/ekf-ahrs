#include "cPDC.h"
#include "utils.h"

cPDC::cPDC(Pdc *pdc)
{
	mPDC = pdc;
}

cPDC::~cPDC()
{
}

void cPDC::enableTransfer(uint32_t controls)
{
	mPDC->PERIPH_PTCR = controls & (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
}

void cPDC::txInit(pdc_packet_t *packet, pdc_packet_t *next_packet)
{
	if (packet) {
		mPDC->PERIPH_TPR = packet->ul_addr;
		mPDC->PERIPH_TCR = packet->ul_size;
	}
	if (next_packet) {
		mPDC->PERIPH_TNPR = next_packet->ul_addr;
		mPDC->PERIPH_TNCR = next_packet->ul_size;
	}
}

void cPDC::sendUART(const char *data, int len)
{
	if(len == -1) {
		len = jani::strlen(data);
	}
	pdc_packet_t pkt;
	pkt.ul_addr = (uint32_t)data;
	pkt.ul_size = len;
	txInit(&pkt);
}

uint32_t cPDC::count()
{
	return mPDC->PERIPH_TCR;
}
