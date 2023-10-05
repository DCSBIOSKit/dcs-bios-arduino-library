#if defined(DCSBIOS_RS485_SLAVE) && defined(DCSBIOS_FOR_STM32DUINO)

namespace DcsBios {
    RS485Slave::RS485Slave() : serial(DCSBIOS_RS485_SERIAL), txen_pin(TXENABLE_PIN) { }

    RS485Slave rs485slave;

    void setup() {
        rs485slave.serial.begin(250000);
    }
    
    void loop() {
        while (rs485slave.serial.available()) {
            parser.processChar(rs485slave.serial.read()); // TODO: Handle RS485 messages.
        }
        PollingInput::pollInputs();
        ExportStreamListener::loopAll();			
    }
    
    /* TODO: This is a work-in-progress.
    
    Here is a brief overview of the DCS-BIOS RS485 protocol extracted from the DcsBiosNgRS485Slave.cpp.inc:

    Main switch() in rxISR for receiving messages from the master:
        1) Initial STATE = UNINITIALIZED.
        2) When any byte is received in this tate, the STATE is set to SYNC and the last_rx_time is set to the current time micros().
        3) If the STATE is SYNC, the following happens:
            a) If the time since the last_rx_time is greater than 500 microseconds, the STATE is set to RX_WAIT_ADDRESS and the switch() continues without a break.
            b) If the time since the last_rx_time is less than 500 microseconds, the STATE is set to SYNC, probably to wait for the next pause in data to re-sync in case something was missed?
        4) If the STATE is RX_WAIT_ADDRESS, the received byte is saved to rx_slave_address and the STATE is set to RX_WAIT_MSGTYPE.
        5) The next byte is saved to rx_msgtype and the STATE is set to RX_WAIT_DATALENGTH.
        6) The next byte is saved to rxtx_len and:
            a) If rxtx_len is zero, the STATE is set to RX_HOST_MESSAGE_COMPLETE and it breaks out.
            b) If rx_slave_address and rx_msgtype are both zero, rx_datatype is set to RXDATA_DCSBIOS_EXPORT, otherwise rx_datatype = RXDATA_IGNORE.
            c) The STATE is set to RX_WAIT_DATA.
        7) rxtx_len is decremented and:
            a) If rx_datatype is RXDATA_DCSBIOS_EXPORT, the received byte is passed to parser.processCharISR().
            b) If rxtx_len is zero, the STATE is set to RX_WAIT_CHECKSUM.
        8) The next byte is ignored and the STATE is set to RX_HOST_MESSAGE_COMPLETE, but since the original code is an ISR the byte must still be consumed by serial.read().

    After the switch, we check if STATE == RX_HOST_MESSAGE_COMPLETE which it will be when rxtx_len has reached zero.
        Now, a few things can happen:
        1) If rx_slave_address == 0, the message should not be answered, so the STATE is set to RX_WAIT_ADDRESS and the above process restarts.
        2) If rx_slave_address == DCSBIOS_RS485_SLAVE, the message should be answered:
            a) If rx_msgtype == 0 && rxtx_len == 0, the message is a poll for DCS-BIOS import data:
                If the messageBuffer is !complete, there is nothing to send yet, so send a delay byte "0" and set STATE = TX_SEND_ZERO_DATALENGTH
                If the messageBuffer is complete, set rxtx_len = messageBuffer.getLength(), send a delay byte "0" and STATE = TX_SEND_DATALENGTH
            b) We have been sent a message that we did not expect, so set last_rx_time = micros() and STATE = SYNC.
        3) If none of the above are true, the message is for someone else, so set STATE = RX_WAIT_ANSWER_DATALENGTH.
    */
    bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
        rs485slave.serial.write(msg); rs485slave.serial.write(' '); rs485slave.serial.write(arg); rs485slave.serial.write('\n'); // TODO: Handle RS485 specifics.
        DcsBios::PollingInput::setMessageSentOrQueued();
        return true;
    }

    void resetAllStates() {
        PollingInput::resetAllStates();
    }
}

#endif // DCSBIOS_RS485_SLAVE && DCSBIOS_FOR_STM32DUINO