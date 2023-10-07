#ifndef _DCSBIOS_STM32_RS485_SLAVE_H_
#define DCSBIOS_STM32_RS485_SLAVE_H_
#if defined(DCSBIOS_RS485_SLAVE) && defined(DCSBIOS_FOR_STM32DUINO)

#include "Arduino.h"
#include "RingBuffer.h"

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
	private:
		serial_t _serial;
		unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    	unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

		static void _rx_complete_irq(serial_t *obj);
		static int _tx_complete_irq(serial_t *obj);
		int availableForWrite(void);
		size_t write(const uint8_t *buffer, size_t size);
		size_t write(uint8_t c);

		void set_txen();
		void clear_txen();
		void tx_delay_byte();
		void tx_byte(uint8_t c);
    public:	
		int txen_pin;

        unsigned int last_rx_time;
        uint8_t rx_slave_address;
		uint8_t rx_msgtype;
        uint8_t rxtx_len;

        enum RxDataType {
			RXDATA_IGNORE,
			RXDATA_DCSBIOS_EXPORT
		};
        RxDataType rx_datatype;
		
        enum State {
			UNINITIALIZED,
			SYNC, // wait for 500 us of no traffic
			RX_WAIT_ADDRESS,
			RX_WAIT_MSGTYPE,
			RX_WAIT_DATALENGTH,
			RX_WAIT_DATA,
			RX_WAIT_CHECKSUM,
			RX_HOST_MESSAGE_COMPLETE,
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
        State state;

        RS485Slave();

		void begin();
    };
}

#endif // DCSBIOS_RS485_SLAVE && DCSBIOS_FOR_STM32DUINO
#endif // _DCSBIOS_STM32_RS485_SLAVE_H_