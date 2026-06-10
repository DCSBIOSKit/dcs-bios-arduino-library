/*
	Host-based tests for the RS485 slave receive path and the DCS-BIOS
	protocol parser. Build and run with `make test` in this directory.

	The include order below mirrors what DcsBios.h does for an RS485
	slave sketch.
*/
#define DCSBIOS_RS485_SLAVE 5
#define TXENABLE_PIN 2

#include "Arduino.h"
#include "test_support.h"

#include "internal/ExportStreamListener.h"
#include "internal/PollingInput.h"
#include "internal/Protocol.h"
#include "internal/Protocol.cpp.inc"
#include "internal/DcsBiosNgRS485Slave.h"
#include "internal/DcsBiosNgRS485Slave.cpp.inc"

// --- output listeners under test (legit test data never contains 'U',
// so any 'U' in a delivered value is committed 0x55 sync garbage) ---
std::vector<std::string> stringValues;
void onStringChange(char* v) { stringValues.push_back(v); }
DcsBios::StringBuffer<6> stringBuffer(0x1000, onStringChange);

std::vector<unsigned int> intValues;
void onIntChange(unsigned int v) { intValues.push_back(v); }
DcsBios::IntegerBuffer intBuffer(0x0408, 0xffff, 0, onIntChange);

std::vector<std::string> negValues;
void onNegChange(char* v) { negValues.push_back(v); }
DcsBios::StringBuffer<6> negBuffer(0x2000, onNegChange);

std::vector<std::string> resetValues;
void onResetChange(char* v) { resetValues.push_back(v); }
DcsBios::StringBuffer<6> resetBuffer(0x3000, onResetChange);

// --- bus simulation helpers ---
void busByte(uint8_t b) {
	UDR0 = b;
	DcsBios::rs485slave.rxISR();
}

void busChunk(uint8_t address, uint8_t msgtype, const std::vector<uint8_t>& payload) {
	busByte(address);
	busByte(msgtype);
	busByte((uint8_t)payload.size());
	for (uint8_t b : payload) busByte(b);
	if (!payload.empty()) busByte(0x72); // checksum (ignored by slaves)
}

void busQuietPeriod() { fake_micros += 1000; }

bool udrieSet() { return UCSR0B & (1 << UDRIE0); }

bool anyValueContainsU(const std::vector<std::string>& values) {
	for (const std::string& v : values)
		if (v.find('U') != std::string::npos) return true;
	return false;
}

int main() {
	DcsBios::setup();

	// =====================================================================
	// Parser unit tests (no bus involved)
	// =====================================================================

	// Negative control: a mid-frame gap without a parser reset commits the
	// next frame's 0x55 sync bytes as data - the historical "UUUU" symptom.
	// This documents the failure mode and proves the assertions can see it.
	{
		ExportStreamBuilder torn;
		torn.sync();
		torn.bytes.insert(torn.bytes.end(), {0x00, 0x20, 0x06, 0x00, 'a', 'b'}); // write @0x2000, count 6, only 2 bytes arrive
		ExportStreamBuilder next;
		next.sync().writeWord(0x0408, 1111).endOfFrame();

		for (uint8_t b : torn.bytes) DcsBios::parser.processChar(b);
		// bytes lost here; parser NOT reset
		for (uint8_t b : next.bytes) DcsBios::parser.processChar(b);
		DcsBios::ExportStreamListener::loopAll();

		EXPECT(negValues.size() == 1 && negValues[0] == "abUUUU",
			"without reset(), a mid-frame gap commits sync bytes as 'U' garbage (bug reproduction)");
	}

	// The cure: reset() at the gap keeps the parser inert until the next
	// sync sequence, so no garbage is ever committed.
	{
		ExportStreamBuilder torn;
		torn.sync();
		torn.bytes.insert(torn.bytes.end(), {0x00, 0x30, 0x06, 0x00, 'a', 'b'}); // write @0x3000, count 6, only 2 bytes arrive
		ExportStreamBuilder next;
		next.sync().writeWord(0x0408, 2222).endOfFrame();

		for (uint8_t b : torn.bytes) DcsBios::parser.processChar(b);
		DcsBios::parser.reset(); // bytes lost here; parser resynchronized
		for (uint8_t b : next.bytes) DcsBios::parser.processChar(b);
		DcsBios::ExportStreamListener::loopAll();

		EXPECT(!anyValueContainsU(resetValues), "with reset(), no sync-byte garbage is committed");
		EXPECT(intValues.size() == 2 && intValues[1] == 2222, "stream parses correctly after reset()");
	}

	// =====================================================================
	// Slave bus state machine tests
	// =====================================================================

	// Bring the slave out of UNINITIALIZED/SYNC: one byte arms the sync
	// detector, then a quiet period unlocks the address state.
	busByte(0x00);
	busQuietPeriod();

	// Happy path: one broadcast chunk carrying a complete export frame.
	{
		// Addresses ascend within a frame, as in the real export stream.
		ExportStreamBuilder frame;
		frame.sync().writeWord(0x0408, 1234).writeString(0x1000, "abcdef").endOfFrame();
		busChunk(0, 0, frame.bytes);
		DcsBios::loop();

		EXPECT(stringValues.size() == 1 && stringValues[0] == "abcdef", "broadcast export data reaches the string callback");
		EXPECT(intValues.size() == 3 && intValues[2] == 1234, "broadcast export data reaches the integer callback");
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ADDRESS, "bus framing intact after broadcast");
	}

	// Large chunk: 126 bytes. The old availableForWrite() guard discarded
	// any chunk >= 65 bytes wholesale, corrupting parser alignment.
	{
		ExportStreamBuilder frame;
		frame.sync().writeWord(0x0408, 4321).writeString(0x1000, "ghijkl");
		frame.write(0x7000, std::vector<uint8_t>(96, 0x00)); // filler to no-listener space
		frame.endOfFrame();
		EXPECT(frame.bytes.size() == 126, "test frame is a single 126-byte chunk");

		busChunk(0, 0, frame.bytes);
		DcsBios::loop();

		EXPECT(stringValues.size() == 2 && stringValues[1] == "ghijkl", "126-byte chunks are processed, not discarded");
		EXPECT(intValues.size() == 4 && intValues[3] == 4321, "integer data in large chunk is processed");
	}

	// Overload: two back-to-back 126-byte chunks with no loop() in between
	// overflow the 128-byte buffer. The slave must keep its bus framing,
	// keep answering polls, and recover by skipping the torn data cleanly.
	{
		ExportStreamBuilder frame1, frame2;
		frame1.sync().writeString(0x1000, "mnopqr").write(0x7000, std::vector<uint8_t>(102, 0x00)).endOfFrame();
		frame2.sync().writeString(0x1000, "stuvwx").write(0x7000, std::vector<uint8_t>(102, 0x00)).endOfFrame();

		size_t stringValuesBefore = stringValues.size();
		busChunk(0, 0, frame1.bytes);
		busChunk(0, 0, frame2.bytes);

		EXPECT(DcsBios::rs485slave.rx_overflow, "buffer overflow is detected");
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ADDRESS, "bus framing survives the overflow");

		// Poll the slave while it is overloaded: it must still answer.
		busChunk(DCSBIOS_RS485_SLAVE, 0, {});
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::TX_SEND_ZERO_DATALENGTH, "slave answers polls while overloaded");
		DcsBios::rs485slave.txcISR(); // delay byte done -> sends zero datalength
		DcsBios::rs485slave.txcISR(); // datalength done -> back to receiving
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ADDRESS, "poll answer completes");

		DcsBios::loop(); // overflow recovery
		EXPECT(!DcsBios::rs485slave.rx_overflow, "overflow flag cleared by recovery");
		EXPECT(DcsBios::parser.incomingDataBuffer.isEmpty(), "torn data discarded by recovery");
		EXPECT(stringValues.size() == stringValuesBefore, "no partial/torn values delivered during overload");

		// The next complete frame is processed normally again.
		ExportStreamBuilder frame3;
		frame3.sync().writeString(0x1000, "yz0123").endOfFrame();
		busChunk(0, 0, frame3.bytes);
		DcsBios::loop();
		EXPECT(stringValues.back() == "yz0123", "clean frame after overload is processed");
		EXPECT(!anyValueContainsU(stringValues), "no sync-byte garbage was ever delivered");
	}

	// Poll answer with a queued input message.
	{
		EXPECT(DcsBios::tryToSendDcsBiosMessage("TEST", "1"), "input message can be queued");
		busChunk(DCSBIOS_RS485_SLAVE, 0, {});
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::TX_SEND_DATALENGTH, "poll with queued message starts an answer");

		std::vector<uint8_t> sent;
		DcsBios::rs485slave.txcISR(); // delay byte done -> sends datalength, enables UDRE
		sent.push_back((uint8_t)UDR0);
		while (udrieSet()) {
			DcsBios::rs485slave.udreISR();
			sent.push_back((uint8_t)UDR0);
		}
		DcsBios::rs485slave.txcISR(); // checksum done -> back to receiving

		std::vector<uint8_t> expected = {7, 0, 'T', 'E', 'S', 'T', ' ', '1', '\n', 0x72};
		EXPECT(sent == expected, "answer is [datalength, msgtype, message, checksum]");
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ADDRESS, "slave returns to receiving after answer");
		EXPECT(!DcsBios::messageBuffer.complete, "message buffer is released after transmission");
	}

	// Messages addressed to other slaves are skipped, not answered.
	{
		busChunk(7, 0, {});
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ANSWER_DATALENGTH, "messages for other slaves await their answer");
		busByte(0); // the other slave answers "nothing to say"
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_ADDRESS, "zero-length answer returns slave to address state");
	}

	// Regression test for the 16-bit last_rx_time truncation: the SYNC
	// quiet-period detection must work across a 65.536 ms micros() window.
	{
		fake_micros = 0x0001FFF0; // just before a 16-bit micros() boundary
		busChunk(DCSBIOS_RS485_SLAVE, 1, {}); // unexpected message type -> slave re-syncs
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::SYNC, "unexpected message puts slave into SYNC");

		fake_micros += 100; // 100 us later, crossing 0x00020000
		busByte(0x42);
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::SYNC,
			"busy bus keeps slave in SYNC across a 16-bit micros() boundary");

		busQuietPeriod();
		busByte(0x00); // first byte after real quiet period is an address
		EXPECT(DcsBios::rs485slave.state == DcsBios::RS485Slave::RX_WAIT_MSGTYPE, "quiet period ends SYNC");
		busByte(0x00); // msgtype
		busByte(0x00); // zero datalength completes the broadcast
	}

	return testResult("test_rs485_slave");
}
