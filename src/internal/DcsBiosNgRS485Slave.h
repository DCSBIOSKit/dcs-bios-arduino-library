#ifndef _DCSBIOS_RS485_SLAVE_H_
#define DCSBIOS_RS485_SLAVE_H_
#ifdef DCSBIOS_RS485_SLAVE

#include "Arduino.h"
#include "RingBuffer.h"

#ifdef DCSBIOS_FOR_STM32
	#include <libmaple/usart.h>
#elif defined(ESP32)
	#include <HardwareSerial.h>
	#include <driver/uart.h>
	#ifndef DCSBIOS_RS485_UART
		#define DCSBIOS_RS485_UART 2
	#endif
	#ifndef DCSBIOS_RS485_RX_PIN
		#define DCSBIOS_RS485_RX_PIN 6
	#endif
	#ifndef DCSBIOS_RS485_TX_PIN
		#define DCSBIOS_RS485_TX_PIN 7
	#endif
#endif

namespace DcsBios {
	ProtocolParser parser;
	DcsBios::RingBuffer<32> messageBuffer;

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		if (messageBuffer.complete) return false; // buffer occupied
		
		messageBuffer.clear();
		const char* c = msg;
		while (*c) {
			messageBuffer.put(*c++);
		}
		messageBuffer.put(' ');
		c = arg;
		while (*c) {
			messageBuffer.put(*c++);
		}
		messageBuffer.put('\n');
		
		messageBuffer.complete = true;
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}
	
	class RS485Slave {
	public:
#ifdef DCSBIOS_FOR_STM32
		volatile uint32_t *udr;
		volatile uint32_t *usart_sr;
		volatile uint32_t *usart_cr1;
		volatile uint32_t *usart_cr2;
		volatile uint32_t *usart_cr3;
		volatile uint32_t *usart_brr;
		
		volatile uint32_t *txen_port;
#elif defined(ESP32)
		HardwareSerial dcsBiosSerial;
		uint8_t uart_num;
		uint8_t txen_pin;
		uint8_t tx_pin;
		uint8_t rx_pin;
#else
		volatile uint8_t *udr;
		volatile uint8_t *ucsra;
		volatile uint8_t *ucsrb;
		volatile uint8_t *ucsrc;
		
		volatile uint8_t *txen_port;
#endif
		volatile uint8_t txen_pin_mask;
		volatile uint8_t rxtx_len;
		volatile uint8_t checksum;
		
		volatile uint8_t state;
		volatile unsigned int last_rx_time;
		volatile uint8_t rx_slave_address;
		volatile uint8_t rx_msgtype;
		
		enum RxDataType {
			RXDATA_IGNORE,
			RXDATA_DCSBIOS_EXPORT
		};
		volatile uint8_t rx_datatype;
		
		enum State {
			UNINITIALIZED,
			SYNC, // wait for 500 us of no traffic
			RX_WAIT_ADDRESS,
			RX_WAIT_MSGTYPE,
			RX_WAIT_DATALENGTH,
			RX_WAIT_DATA,
			RX_WAIT_CHECKSUM,
			RX_HOST_MESSAGE_COMPLETE, // used as temporary state to simplify program flow
			RX_WAIT_ANSWER_DATALENGTH,
			RX_WAIT_ANSWER_MSGTYPE,
			RX_WAIT_ANSWER_DATA,
			RX_WAIT_ANSWER_CHECKSUM,
			TX_SEND_ZERO_DATALENGTH,
			TX_ZERO_DATALENGTH_SENT,
			TX_SEND_DATALENGTH,
			TX_DATALENGTH_SENT,
			TX,
			TX_CHECKSUM_SENT
		};
		
		#ifdef ESP32
		void rxISR(uint8_t c);
		void onReceive(void);
		#else
		void rxISR();
		#endif
		void udreISR();
		void txcISR();
		
#ifdef DCSBIOS_FOR_STM32
		volatile bool tx_enabled = false, rx_enabled = true, udrie_set = false;
		inline void set_txen() {
			*usart_cr1 &= ~((1<<USART_CR1_RE_BIT) | (1<<USART_CR1_RXNEIE_BIT)); 
			*txen_port |= txen_pin_mask;
			*usart_cr1 |= (1<<USART_CR1_TE_BIT) | (1<<USART_CR1_TCIE_BIT);
			rx_enabled = false;
			tx_enabled = true;
		};
		inline void clear_txen() {
			*usart_cr1 &= ~((1<<USART_CR1_TE_BIT) | (1<<USART_CR1_TCIE_BIT));
			*txen_port &= ~txen_pin_mask;
			*usart_cr1 |= (1<<USART_CR1_RE_BIT) | (1<<USART_CR1_RXNEIE_BIT);
			rx_enabled = true;
			tx_enabled = false;
		};
		inline void tx_byte(uint8_t c) { set_txen(); volatile uint32_t temp = *usart_sr; *udr = c; } //*usart_sr &= ~(1<<USART_SR_TC_BIT); }
		inline void tx_delay_byte() { *usart_cr1 |= (1<<USART_CR1_TE_BIT) | (1<<USART_CR1_TCIE_BIT); *udr = 0; tx_enabled = true; }
		inline void set_udrie() { *usart_cr1 |= (1<<USART_CR1_TXEIE_BIT); udrie_set = true ; }
		inline void clear_udrie() { *usart_cr1 &= ~(1<<USART_CR1_TXEIE_BIT); udrie_set = false; }
		RS485Slave(usart_dev *usart, uint32_t txen_pin);
		void UsartIRQ();
#elif defined(ESP32)
		inline void set_txen() {/* Noop, handled by UART on ESP32 */}
		inline void clear_txen() {/* Noop, handled by UART on ESP32 */}
		inline void tx_byte(uint8_t c) {
			dcsBiosSerial.write(c);
		}
		inline void tx_delay_byte() { tx_byte(0); }
		inline void set_udrie() {
			#warning "Not implemented"
		}
		inline void clear_udrie() {
			#warning "Not implemented"
		}

		RS485Slave(uint8_t uart_num, uint8_t txen_pin, uint8_t tx_pin, uint8_t rx_pin);
#else
		inline void set_txen() { *ucsrb &= ~((1<<RXEN0) | (1<<RXCIE0)); *txen_port |= txen_pin_mask; *ucsrb |= (1<<TXEN0) | (1<<TXCIE0); };
		inline void clear_txen() { *ucsrb &= ~((1<<TXEN0) | (1<<TXCIE0)); *txen_port &= ~txen_pin_mask; *ucsrb |= (1<<RXEN0) | (1<<RXCIE0); };
		inline void tx_byte(uint8_t c) { set_txen(); *udr = c; *ucsra |= (1<<TXC0); }
		inline void tx_delay_byte() { *ucsrb |= ((1<<TXEN0) | (1<<TXCIE0)); *udr = 0; }
		inline void set_udrie() { *ucsrb |= (1<<UDRIE0); }
		inline void clear_udrie() { *ucsrb &= ~(1<<UDRIE0); }
		
		RS485Slave(volatile uint8_t *udr, volatile uint8_t *ucsra, volatile uint8_t *ucsrb, volatile uint8_t *ucsrc, uint8_t txen_pin);
#endif
	};
}

#endif // DCSBIOS_RS485_SLAVE
#endif // _DCSBIOS_RS485_SLAVE_H_