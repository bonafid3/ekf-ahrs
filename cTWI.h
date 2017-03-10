#ifndef __CTWI_H__
#define __CTWI_H__

#include <sam3s4b.h>

#define DISABLE   0
#define ENABLE    1
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
#define false     0
#define true      1
#endif
#endif
#define PASS      0
#define FAIL      1
#define LOW       0
#define HIGH      1

#define TWI_SUCCESS              0
#define TWI_INVALID_ARGUMENT     1
#define TWI_ARBITRATION_LOST     2
#define TWI_NO_CHIP_FOUND        3
#define TWI_RECEIVE_OVERRUN      4
#define TWI_RECEIVE_NACK         5
#define TWI_SEND_OVERRUN         6
#define TWI_SEND_NACK            7
#define TWI_BUSY                 8
#define TWI_ERROR_TIMEOUT        9

#define I2C_FAST_MODE_SPEED  400000
#define TWI_CLK_DIVIDER      2
#define TWI_CLK_CALC_ARGU    4
#define TWI_CLK_DIV_MAX      0xFF
#define TWI_CLK_DIV_MIN      7

typedef struct twi_options {
	//! MCK for TWI.
	uint32_t master_clk;
	//! The baud rate of the TWI bus.
	uint32_t speed;
	//! The desired address.
	uint8_t chip;
	//! SMBUS mode (set 1 to use SMBUS quick command, otherwise don't).
	uint8_t smbus;
} twi_options_t;

typedef struct twi_packet {
	//! TWI address/commands to issue to the other chip (node).
	uint8_t addr[3];
	//! Length of the TWI data address segment (1-3 bytes).
	uint32_t addr_length;
	//! Where to find the data to be transferred.
	void *buffer;
	//! How many bytes do we want to transfer.
	uint32_t length;
	//! TWI chip address to communicate with.
	uint8_t chip;
} twi_packet_t;

#define TWI_TIMEOUT              15000

class cTWI
{
public:
	cTWI(Twi *twi);
	~cTWI();
	
	uint32_t init_master(const twi_options_t *p_opt);
	void enableMasterMode();

	void reset();
	
	uint32_t makeAddr(const uint8_t *addr, int len);
	uint32_t setSpeed(uint32_t ul_speed, uint32_t ul_mck);
	
	uint32_t masterWriteByte(uint8_t daddr, uint8_t iaddr, uint8_t data);
	uint32_t masterWriteWord(uint8_t daddr, uint8_t iaddr, uint16_t data);
	uint32_t masterWrite(twi_packet_t *p_packet);
	uint32_t probe(uint8_t uc_slave_addr);
	
	uint32_t masterSetBit(uint8_t daddr, uint8_t iaddr, uint8_t bit);
	uint32_t masterClearBit(uint8_t daddr, uint8_t iaddr, uint8_t bit);
	void setDeviceAddress(uint8_t addr);
	
	uint8_t masterReadByte(uint8_t daddr, uint8_t iaddr);
	
	uint16_t masterReadWord(uint8_t daddr, uint8_t iaddr);
	uint32_t masterRead(twi_packet_t *p_packet);
	
private:
	Twi *mTWI;
	
};

#endif
