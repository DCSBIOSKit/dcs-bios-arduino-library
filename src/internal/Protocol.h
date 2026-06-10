#ifndef __DCSBIOS_PROTOCOL_H
#define __DCSBIOS_PROTOCOL_H

#define DCSBIOS_STATE_WAIT_FOR_SYNC 0
#define DCSBIOS_STATE_ADDRESS_LOW 1
#define DCSBIOS_STATE_ADDRESS_HIGH 2
#define DCSBIOS_STATE_COUNT_LOW 3
#define DCSBIOS_STATE_COUNT_HIGH 4
#define DCSBIOS_STATE_DATA_LOW 5
#define DCSBIOS_STATE_DATA_HIGH 6

/*
	RS485 slaves buffer export data in the receive interrupt and process it
	from loop(), so they get a larger default buffer to ride out slow loop()
	iterations (e.g. display refreshes). Can be overridden from the sketch.
*/
#ifndef DCSBIOS_INCOMING_DATA_BUFFER_SIZE
	#ifdef DCSBIOS_RS485_SLAVE
		#define DCSBIOS_INCOMING_DATA_BUFFER_SIZE 128
	#else
		#define DCSBIOS_INCOMING_DATA_BUFFER_SIZE 64
	#endif
#endif

#include "ExportStreamListener.h"
#include "RingBuffer.h"

namespace DcsBios {

	class ProtocolParser {
		private:
			volatile unsigned char state;
			volatile unsigned int address;
			volatile unsigned int count;
			volatile unsigned int data;
			volatile unsigned char sync_byte_count;
			
			ExportStreamListener* startESL;
			volatile bool processingData;
		public:
			RingBuffer<DCSBIOS_INCOMING_DATA_BUFFER_SIZE> incomingDataBuffer;
			
			void processChar(unsigned char c);
			void processCharISR(unsigned char c);
			void reset();
			ProtocolParser();
	};
}

#endif