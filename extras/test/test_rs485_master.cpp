/*
	Host-based tests for the RS485 master: broadcast chunking and
	PC-connection overflow behavior. Build and run with `make test`.
*/
#define __AVR_ATmega2560__
#define DCSBIOS_RS485_MASTER
#define UART1_TXENABLE_PIN 22

#include "Arduino.h"
#include "test_support.h"

#include "internal/ExportStreamListener.h"
#include "internal/PollingInput.h"
#include "internal/Protocol.h"
#include "internal/DcsBiosNgRS485Master.h"
#include "internal/DcsBiosNgRS485Master.cpp.inc"

void pcByte(uint8_t b) {
	UDR0 = b;
	DcsBios::uart0.rxISR();
}

bool uart1UdrieSet() { return UCSR1B & (1 << UDRIE0); }

int main() {
	DcsBios::setup();

	// Broadcast chunking: data from the PC is sent in chunks no larger
	// than DCSBIOS_RS485_MAX_CHUNK_LENGTH, in order, fully framed.
	{
		for (int i = 0; i < 200; i++) pcByte((uint8_t)i);
		EXPECT(DcsBios::uart1.exportData.getLength() == 200, "PC data is buffered for the RS485 bus");

		std::vector<uint8_t> sent;
		DcsBios::uart1.loop(); // sends the address byte, enables UDRE
		sent.push_back((uint8_t)UDR1);
		while (uart1UdrieSet()) {
			DcsBios::uart1.udreISR();
			sent.push_back((uint8_t)UDR1);
		}
		DcsBios::uart1.txcISR(); // checksum done -> IDLE

		EXPECT(sent.size() == 3 + 64 + 1, "chunk is address, msgtype, datalength, data, checksum");
		EXPECT(sent[0] == 0 && sent[1] == 0, "broadcast chunks use address 0, msgtype 0");
		EXPECT(sent[2] == 64, "chunks are capped at DCSBIOS_RS485_MAX_CHUNK_LENGTH");
		bool dataInOrder = true;
		for (int i = 0; i < 64; i++) {
			if (sent[3 + i] != (uint8_t)i) dataInOrder = false;
		}
		EXPECT(dataInOrder, "chunk carries the buffered bytes in order");
		EXPECT(DcsBios::uart1.exportData.getLength() == 136, "remaining data stays buffered for the next chunk");
		EXPECT(DcsBios::uart1.state == DcsBios::RS485Master::IDLE, "master returns to IDLE after the chunk");
	}

	// Overflow: when the PC out-paces the bus, the buffer must saturate
	// cleanly (drop new bytes) instead of lapping itself and scrambling
	// already-buffered data.
	{
		while (DcsBios::uart1.exportData.isNotEmpty()) DcsBios::uart1.exportData.get();

		for (int i = 0; i < 400; i++) pcByte((uint8_t)i);
		EXPECT(DcsBios::uart1.exportData.isFull(), "buffer saturates under sustained PC traffic");
		EXPECT(DcsBios::uart1.exportData.getLength() == 255, "buffer never laps itself");

		bool intact = true;
		for (int i = 0; i < 255; i++) {
			if (DcsBios::uart1.exportData.get() != (uint8_t)i) intact = false;
		}
		EXPECT(intact, "already-buffered data is untouched by the overflow");
		EXPECT(DcsBios::uart1.exportData.isEmpty(), "buffer fully drains after overflow");
	}

	return testResult("test_rs485_master");
}
