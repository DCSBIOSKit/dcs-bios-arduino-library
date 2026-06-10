#ifndef _DCSBIOS_TEST_SUPPORT_H_
#define _DCSBIOS_TEST_SUPPORT_H_

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

unsigned long fake_micros = 0;
volatile uint8_t fake_txen_port = 0;
volatile uint8_t UDR0, UCSR0A, UCSR0B, UCSR0C, UBRR0H, UBRR0L;
volatile uint8_t UDR1, UCSR1A, UCSR1B, UCSR1C, UBRR1H, UBRR1L;
volatile uint8_t UDR2, UCSR2A, UCSR2B, UCSR2C, UBRR2H, UBRR2L;
volatile uint8_t UDR3, UCSR3A, UCSR3B, UCSR3C, UBRR3H, UBRR3L;
volatile uint8_t PRR0, PRR1;
volatile uint8_t PORTH, PORTB;

static int failures = 0;

#define EXPECT(cond, msg) do { \
	if (cond) { \
		printf("PASS: %s\n", msg); \
	} else { \
		printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
		failures++; \
	} \
} while (0)

inline int testResult(const char* suite) {
	if (failures) {
		printf("%s: %d FAILURE(S)\n", suite, failures);
		return 1;
	}
	printf("%s: all tests passed\n", suite);
	return 0;
}

/*
	Builds a DCS-BIOS export stream: sync sequence followed by write blocks.
*/
struct ExportStreamBuilder {
	std::vector<uint8_t> bytes;

	ExportStreamBuilder& sync() {
		for (int i = 0; i < 4; i++) bytes.push_back(0x55);
		return *this;
	}
	ExportStreamBuilder& write(uint16_t address, const std::vector<uint8_t>& data) {
		bytes.push_back(address & 0xff);
		bytes.push_back(address >> 8);
		bytes.push_back(data.size() & 0xff);
		bytes.push_back(data.size() >> 8);
		bytes.insert(bytes.end(), data.begin(), data.end());
		return *this;
	}
	ExportStreamBuilder& writeString(uint16_t address, const char* s) {
		return write(address, std::vector<uint8_t>(s, s + strlen(s)));
	}
	ExportStreamBuilder& writeWord(uint16_t address, uint16_t value) {
		return write(address, { (uint8_t)(value & 0xff), (uint8_t)(value >> 8) });
	}
	// Every real export frame ends with a write to the update counter.
	ExportStreamBuilder& endOfFrame(uint16_t counter = 1) {
		return writeWord(0xfffe, counter);
	}
};

#endif
