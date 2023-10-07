#if defined(DCSBIOS_RS485_SLAVE) && defined(DCSBIOS_FOR_STM32DUINO)

namespace DcsBios {
    RS485Slave::RS485Slave() : serial(DCSBIOS_RS485_SERIAL), txen_pin(TXENABLE_PIN) {
        state = UNINITIALIZED;
    }

    RS485Slave rs485slave;

    void setup() {
        // TODO: Initialize UART
    }
    
    void loop() {
		PollingInput::pollInputs();
		ExportStreamListener::loopAll();
	}

    void resetAllStates() {
        PollingInput::resetAllStates();
    }
}

#endif // DCSBIOS_RS485_SLAVE && DCSBIOS_FOR_STM32DUINO