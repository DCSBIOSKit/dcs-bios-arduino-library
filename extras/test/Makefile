CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -g
INCLUDES := -Istub -I../../src

SOURCES := $(wildcard ../../src/internal/*.h ../../src/internal/*.cpp.inc)
TESTS := test_rs485_slave test_rs485_master

.PHONY: all test clean

all: $(TESTS)

test_rs485_slave: test_rs485_slave.cpp test_support.h stub/Arduino.h $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $<

test_rs485_master: test_rs485_master.cpp test_support.h stub/Arduino.h $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $<

test: all
	./test_rs485_slave
	./test_rs485_master

clean:
	rm -f $(TESTS)
