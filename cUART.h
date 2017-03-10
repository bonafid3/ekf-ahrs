#ifndef __CUART_H__
#define __CUART_H__

#include <sam3s4b.h>

/* UART internal div factor for sampling */
#define UART_MCK_DIV             16
/* Div factor to get the maximum baud rate */
#define UART_MCK_DIV_MIN_FACTOR  1
/* Div factor to get the minimum baud rate */
#define UART_MCK_DIV_MAX_FACTOR  65535

/*! \brief Option list for UART peripheral initialization */
typedef struct sam_uart_opt {
	/** MCK for UART */
	uint32_t ul_mck;
	/** Expected baud rate */
	uint32_t ul_baudrate;
	/** Initialize value for UART mode register */
	uint32_t ul_mode;
} sam_uart_opt_t;


class cUART
{
public:
	cUART(Uart *uart_base);
	~cUART();

	uint32_t init(const sam_uart_opt_t *opts);
	void enableTX();
	void enableRX();
	
	void enableInterrupt(uint32_t source);
	void disableInterrupt(uint32_t source);
	
	uint32_t status();
	
	uint32_t txReady();
	uint32_t rxReady();
	
	uint32_t write(uint8_t byte);
	uint32_t read(uint8_t &byte);
	
	
private:
	Uart *mUART;
};

#endif
