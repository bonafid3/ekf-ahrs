#include "cTWI.h"


cTWI::cTWI(Twi *twi)
{
	mTWI = twi;
}


cTWI::~cTWI()
{}

uint32_t cTWI::init_master(const twi_options_t *p_opt)
{
	uint32_t status = TWI_SUCCESS;

	// Disable TWI interrupts
	mTWI->TWI_IDR = ~0UL;

	// Dummy read in status register
	mTWI->TWI_SR;

	// Reset TWI peripheral
	reset();

	// master mode
	enableMasterMode();

	// Select the speed
	if (setSpeed(p_opt->speed, p_opt->master_clk) == FAIL) {
		// The desired speed setting is rejected
		status = TWI_INVALID_ARGUMENT;
	}

	if (p_opt->smbus == 1) {
		mTWI->TWI_CR = TWI_CR_QUICK;
	}

	return status;
}

void cTWI::reset()
{
	// Set SWRST bit to reset TWI peripheral
	mTWI->TWI_CR = 0;
	mTWI->TWI_CR = TWI_CR_SWRST;
	mTWI->TWI_RHR;
}

void cTWI::enableMasterMode()
{
	// Set Master Disable bit and Slave Disable bit
	mTWI->TWI_CR = TWI_CR_MSDIS;
	mTWI->TWI_CR = TWI_CR_SVDIS;

	// Set Master Enable bit
	mTWI->TWI_CR = TWI_CR_MSEN;
}


uint32_t cTWI::setSpeed(uint32_t ul_speed, uint32_t ul_mck)
{
	uint32_t ckdiv = 0;
	uint32_t c_lh_div;

	if (ul_speed > I2C_FAST_MODE_SPEED) {
		return FAIL;
	}

	c_lh_div = ul_mck / (ul_speed * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;

	// cldiv must fit in 8 bits, ckdiv must fit in 3 bits
	while ((c_lh_div > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
		// Increase clock divider
		ckdiv++;
		// Divide cldiv value
		c_lh_div /= TWI_CLK_DIVIDER;
	}

	// set clock waveform generator register
	mTWI->TWI_CWGR =
	TWI_CWGR_CLDIV(c_lh_div) | TWI_CWGR_CHDIV(c_lh_div) |
	TWI_CWGR_CKDIV(ckdiv);

	return PASS;
}

uint32_t cTWI::makeAddr(const uint8_t *addr, int len)
{
	uint32_t val;

	if (len == 0)
	return 0;

	val = addr[0];
	if (len > 1) {
		val <<= 8;
		val |= addr[1];
	}
	if (len > 2) {
		val <<= 8;
		val |= addr[2];
	}
	return val;
}


uint32_t cTWI::masterWrite(twi_packet_t *p_packet)
{
	uint32_t status;
	volatile uint32_t cnt = p_packet->length;
	uint8_t *buffer = (uint8_t *)p_packet->buffer;

	/* Check argument */
	if (cnt == 0) {
		return TWI_INVALID_ARGUMENT;
	}

	/* Set write mode, slave address and 3 internal address byte lengths */
	mTWI->TWI_MMR = 0;
	mTWI->TWI_MMR = TWI_MMR_DADR(p_packet->chip) |
	((p_packet->addr_length << TWI_MMR_IADRSZ_Pos) &
	TWI_MMR_IADRSZ_Msk);

	/* Set internal address for remote chip */
	mTWI->TWI_IADR = 0;
	mTWI->TWI_IADR = makeAddr(p_packet->addr, p_packet->addr_length);

	/* Send all bytes */
	while (cnt > 0) {
		status = mTWI->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (!(status & TWI_SR_TXRDY))
		{
			continue;
		}
		mTWI->TWI_THR = *buffer++;

		cnt--;
	}

	while (1)
	{
		status = mTWI->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (status & TWI_SR_TXRDY) {
			break;
		}
	}

	mTWI->TWI_CR = TWI_CR_STOP;

	while (!(mTWI->TWI_SR & TWI_SR_TXCOMP)) {
	}

	return TWI_SUCCESS;
}

uint32_t cTWI::probe(uint8_t uc_slave_addr)
{
	twi_packet_t packet;
	uint8_t data = 0;

	/* Data to send */
	packet.buffer = &data;
	/* Data length */
	packet.length = 1;
	/* Slave chip address */
	packet.chip = (uint32_t) uc_slave_addr;
	/* Internal chip address */
	packet.addr[0] = 0;
	/* Address length */
	packet.addr_length = 0;

	/* Perform a master write access */
	return masterWrite(&packet);
}

uint32_t cTWI::masterSetBit(uint8_t daddr, uint8_t iaddr, uint8_t bit)
{
	uint8_t data = masterReadByte(daddr, iaddr);
	data |= (1<<bit);
	return masterWriteByte(daddr, iaddr, data);
}

uint32_t cTWI::masterClearBit(uint8_t daddr, uint8_t iaddr, uint8_t bit)
{
	uint8_t data = masterReadByte(daddr, iaddr);
	data &= ~(1<<bit);
	return masterWriteByte(daddr, iaddr, data);
}

uint8_t cTWI::masterReadByte(uint8_t daddr, uint8_t iaddr)
{
	uint8_t buffer;
	twi_packet_t pkt;
	pkt.addr[0]=iaddr;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&buffer;
	pkt.chip=daddr;
	pkt.length=1;
	
	bool res = (masterRead(&pkt) == TWI_SUCCESS);
	
	return buffer;	
}

uint16_t cTWI::masterReadWord(uint8_t daddr, uint8_t iaddr)
{
	uint8_t buffer[2];
	twi_packet_t pkt;
	pkt.addr[0]=iaddr;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&buffer;
	pkt.chip=daddr;
	pkt.length=2;
	
	masterRead(&pkt);
	
	uint16_t res = (buffer[0] << 8) | buffer[1];
	return res;
}

uint32_t cTWI::masterWriteByte(uint8_t daddr, uint8_t iaddr, uint8_t data)
{
	twi_packet_t pkt;
	pkt.addr[0]=iaddr;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&data;
	pkt.chip=daddr;
	pkt.length=1;
	
	return masterWrite(&pkt);
}

uint32_t cTWI::masterWriteWord(uint8_t daddr, uint8_t iaddr, uint16_t data)
{
	twi_packet_t pkt;
	uint8_t buffer[2] = {0};
	buffer[0] = data & 0xff;
	buffer[1] = (data >> 8) & 0xff;
	pkt.addr[0]=iaddr;
	pkt.addr[1]=0;
	pkt.addr[2]=0;
	pkt.addr_length=1;
	pkt.buffer=&buffer[0];
	pkt.chip=daddr;
	pkt.length=2;
	
	return masterWrite(&pkt);
}



uint32_t cTWI::masterRead(twi_packet_t *p_packet)
{
	uint32_t status;
	uint32_t cnt = p_packet->length;
	uint8_t *buffer = (uint8_t *)p_packet->buffer;
	uint8_t stop_sent = 0;
	uint32_t timeout = TWI_TIMEOUT;
	
	// Check argument
	if (cnt == 0) {
		return TWI_INVALID_ARGUMENT;
	}

	// Set read mode, slave address and 3 internal address byte lengths
	mTWI->TWI_MMR = 0;
	mTWI->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(p_packet->chip) |
	((p_packet->addr_length << TWI_MMR_IADRSZ_Pos) &
	TWI_MMR_IADRSZ_Msk);

	// Set internal address for remote chip
	mTWI->TWI_IADR = 0;
	mTWI->TWI_IADR = makeAddr(p_packet->addr, p_packet->addr_length);

	// Send a START condition
	if (cnt == 1) {
		mTWI->TWI_CR = TWI_CR_START | TWI_CR_STOP;
		stop_sent = 1;
		} else {
		mTWI->TWI_CR = TWI_CR_START;
		stop_sent = 0;
	}

	while (cnt > 0) {
		status = mTWI->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (!timeout--) {
			return TWI_ERROR_TIMEOUT;
		}
		
		// Last byte ?
		if (cnt == 1  && !stop_sent) {
			mTWI->TWI_CR = TWI_CR_STOP;
			stop_sent = 1;
		}

		if (!(status & TWI_SR_RXRDY)) {
			continue;
		}
		*buffer++ = mTWI->TWI_RHR;

		cnt--;
		timeout = TWI_TIMEOUT;
	}

	while (!(mTWI->TWI_SR & TWI_SR_TXCOMP)) {
	}

	mTWI->TWI_SR;

	return TWI_SUCCESS;
}
