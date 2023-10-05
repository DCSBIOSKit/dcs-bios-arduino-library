#ifndef _DCSBIOS_STM32_RS485_SLAVE_H_
#define DCSBIOS_STM32_RS485_SLAVE_H_
#if defined(DCSBIOS_RS485_SLAVE) && defined(DCSBIOS_FOR_STM32DUINO)

#include "Arduino.h"
#include "RingBuffer.h"

namespace DcsBios {
    ProtocolParser parser;

    class RS485Slave {
    public:
        HardwareSerial serial;
        volatile int txen_pin;

        RS485Slave();
    };
}

#endif // DCSBIOS_RS485_SLAVE && DCSBIOS_FOR_STM32DUINO
#endif // _DCSBIOS_STM32_RS485_SLAVE_H_