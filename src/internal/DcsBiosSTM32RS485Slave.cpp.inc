#if defined(DCSBIOS_RS485_SLAVE) && defined(DCSBIOS_FOR_STM32DUINO)

namespace DcsBios {
    RS485Slave::RS485Slave() : txen_pin(TXENABLE_PIN) {
        // Set pins
        int pins[] = {DCSBIOS_RS485_SERIAL};
        _serial.pin_rx = digitalPinToPinName(pins[0]);
        _serial.pin_tx = digitalPinToPinName(pins[1]);
        _serial.pin_rts = NC;
        _serial.pin_cts = NC;
        _serial.rx_buff = _rx_buffer;
        _serial.rx_head = 0;
        _serial.rx_tail = 0;
        _serial.tx_buff = _tx_buffer;
        _serial.tx_head = 0;
        _serial.tx_tail = 0;

        state = UNINITIALIZED;

        pinMode(txen_pin, OUTPUT);
    }

    RS485Slave rs485slave;

    void RS485Slave::begin() {
        uart_init(&_serial, (uint32_t)250000, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1);
        uart_attach_rx_callback(&_serial, _rx_complete_irq);
    }

    int RS485Slave::availableForWrite(void)
    {
        tx_buffer_index_t head = _serial.tx_head;
        tx_buffer_index_t tail = _serial.tx_tail;

        if (head >= tail) {
            return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
        }
        return tail - head - 1;
    }

    size_t RS485Slave::write(const uint8_t *buffer, size_t size) {
        size_t size_intermediate;
        size_t ret = size;
        size_t available = availableForWrite();
        size_t available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;

        while (!availableForWrite()) {
            // nop, the interrupt handler will free up space for us
        }

        // HAL doesn't manage rollover, so split transfer till end of TX buffer
        // Also, split transfer according to available space in buffer
        while ((size > available_till_buffer_end) || (size > available)) {
            size_intermediate = min(available, available_till_buffer_end);
            write(buffer, size_intermediate);
            size -= size_intermediate;
            buffer += size_intermediate;
            available = availableForWrite();
            available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;
        }

        // Copy data to buffer. Take into account rollover if necessary.
        if (_serial.tx_head + size <= SERIAL_TX_BUFFER_SIZE) {
            memcpy(&_serial.tx_buff[_serial.tx_head], buffer, size);
            size_intermediate = size;
        } else {
            // memcpy till end of buffer then continue memcpy from beginning of buffer
            size_intermediate = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;
            memcpy(&_serial.tx_buff[_serial.tx_head], buffer, size_intermediate);
            memcpy(&_serial.tx_buff[0], buffer + size_intermediate,
                        size - size_intermediate);
        }

        // Data are copied to buffer, move head pointer accordingly
        _serial.tx_head = (_serial.tx_head + size) % SERIAL_TX_BUFFER_SIZE;

        // Transfer data with HAL only is there is no TX data transfer ongoing
        // otherwise, data transfer will be done asynchronously from callback
        if (!serial_tx_active(&_serial)) {
            // note: tx_size correspond to size of HAL data transfer,
            // not the total amount of data in the buffer.
            // To compute size of data in buffer compare head and tail
            _serial.tx_size = size_intermediate;
            uart_attach_tx_callback(&_serial, _tx_complete_irq, size_intermediate);
        }

        /* There is no real error management so just return transfer size requested*/
        return ret;
    }

    size_t RS485Slave::write(uint8_t c) {
        uint8_t buff = c;
        return write(&buff, 1);
    }

    void RS485Slave::_rx_complete_irq(serial_t *obj) {
        uint8_t c;

        if (uart_getc(obj, &c) == 0) {
                switch(rs485slave.state) {
                case UNINITIALIZED:
                    rs485slave.last_rx_time = micros();
                    rs485slave.state = SYNC;
                break;
                
                case SYNC:
                    if (micros() - rs485slave.last_rx_time < 500) {
                        rs485slave.last_rx_time = micros();
                        break;
                    } else {
                        rs485slave.state = RX_WAIT_ADDRESS;
                    }
                // no break statement here, we want it to fall through to the next case!
                
                case RX_WAIT_ADDRESS:
                    rs485slave.rx_slave_address = c;
                    rs485slave.state = RX_WAIT_MSGTYPE;
                break;
                
                case RX_WAIT_MSGTYPE:
                    rs485slave.rx_msgtype = c;
                    rs485slave.state = RX_WAIT_DATALENGTH;
                break;
                
                case RX_WAIT_DATALENGTH:
                    rs485slave.rxtx_len = c;

                    if (rs485slave.rxtx_len > parser.availableBufferSpace()) {
                        rs485slave.last_rx_time = micros();
                        rs485slave.state = SYNC;
                    }
                    
                    if (rs485slave.rxtx_len == 0) {
                        rs485slave.state = RX_HOST_MESSAGE_COMPLETE;
                        break;
                    }
                    
                    if (rs485slave.rx_slave_address == 0 && rs485slave.rx_msgtype == 0) {
                        rs485slave.rx_datatype = RXDATA_DCSBIOS_EXPORT;
                    } else {
                        rs485slave.rx_datatype = RXDATA_IGNORE;
                    }
                    
                    rs485slave.state = RX_WAIT_DATA;
                break;
                
                case RX_WAIT_DATA:
                    rs485slave.rxtx_len--;
                    if (rs485slave.rx_datatype == RXDATA_DCSBIOS_EXPORT) {
                        parser.processCharISR(c);
                    }
                    if (rs485slave.rxtx_len == 0) {
                        rs485slave.state = RX_WAIT_CHECKSUM;
                    }
                break;
                
                case RX_WAIT_CHECKSUM:
                    // ignore checksum
                    rs485slave.state = RX_HOST_MESSAGE_COMPLETE;
                break;
                
                case RX_WAIT_ANSWER_DATALENGTH:
                    rs485slave.rxtx_len = c;
                    if (rs485slave.rxtx_len == 0) {
                        rs485slave.state = RX_WAIT_ADDRESS;
                    } else {
                        rs485slave.state = RX_WAIT_ANSWER_MSGTYPE;
                    }
                break;
                
                case RX_WAIT_ANSWER_MSGTYPE:
                    rs485slave.rx_msgtype = c;
                    rs485slave.state = RX_WAIT_ANSWER_DATA;
                break;
                
                case RX_WAIT_ANSWER_DATA:
                    rs485slave.rxtx_len--;
                    if (rs485slave.rxtx_len == 0) {
                        rs485slave.state = RX_WAIT_ANSWER_CHECKSUM;
                    }
                break;
                
                case RX_WAIT_ANSWER_CHECKSUM:
                    // ignore checksum
                    rs485slave.state = RX_WAIT_ADDRESS;
                break;
            }
            
            if (rs485slave.state == RX_HOST_MESSAGE_COMPLETE) {
                if (rs485slave.rx_slave_address == 0) {
                    // messages to address zero must not be answered by anyone
                    rs485slave.state = RX_WAIT_ADDRESS;
                } else if (rs485slave.rx_slave_address == DCSBIOS_RS485_SLAVE) {
                    // we have to answer this message
                    if (rs485slave.rx_msgtype == 0 && rs485slave.rxtx_len == 0) {
                        // we have just been polled for DCS-BIOS import data...
                        if (!messageBuffer.complete) {
                            // ...but we have nothing to say
                            rs485slave.tx_delay_byte();
                            rs485slave.state = TX_SEND_ZERO_DATALENGTH;
                        } else {
                            // ...and we have something to transmit
                            rs485slave.rxtx_len = messageBuffer.getLength();
                            rs485slave.tx_delay_byte();
                            rs485slave.state = TX_SEND_DATALENGTH;
                        }
                    } else {
                        // we have been sent a message that we did not expect
                        rs485slave.last_rx_time = micros();
                        rs485slave.state = SYNC;
                    }
                } else {
                    // this message is for someone else
                    rs485slave.state = RX_WAIT_ANSWER_DATALENGTH;
                }
            }

            // This is original code from HardwareSerial.cpp, may not be needed.
            rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

            if (i != obj->rx_tail) {
                obj->rx_buff[obj->rx_head] = c;
                obj->rx_head = i;
            }
        }
    }

    // Actual transmission is triggered by tx_delay_byte() from _rx_complete_irq() above.
    // TODO: If this doesn't work, try going deeper with extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    int RS485Slave::_tx_complete_irq(serial_t *obj) {
        rs485slave.clear_txen();

        switch (rs485slave.state) {
			case TX_DATALENGTH_SENT:
				rs485slave.tx_byte(0); // MSGTYPE
				rs485slave.state = TX;
			break;
			
			case TX:
				if (rs485slave.rxtx_len == 0) {
					rs485slave.tx_byte(0x72); // TODO: transmit actual checksum
					rs485slave.state = TX_CHECKSUM_SENT;
				} else {
					rs485slave.rxtx_len--;
					rs485slave.tx_byte(messageBuffer.get());
				}
			break;

            case TX_SEND_ZERO_DATALENGTH:
				rs485slave.tx_byte(0);
				rs485slave.state = TX_ZERO_DATALENGTH_SENT;
			break;
			
			case TX_SEND_DATALENGTH:
				rs485slave.tx_byte(rs485slave.rxtx_len);
				rs485slave.state = TX_DATALENGTH_SENT;
			break;
			
			case TX_ZERO_DATALENGTH_SENT:
				rs485slave.state = RX_WAIT_ADDRESS;
			break;
			
			case TX_CHECKSUM_SENT:
				rs485slave.state = RX_WAIT_ADDRESS;
				messageBuffer.clear();
				messageBuffer.complete = false;
			break;
		}

        return 0;
    }

    void RS485Slave::set_txen() {
        digitalWrite(rs485slave.txen_pin, HIGH);
    }

    void RS485Slave::clear_txen() {
        digitalWrite(rs485slave.txen_pin, LOW);
    }

    void RS485Slave::tx_delay_byte() {
        write(0);
    }

    void RS485Slave::tx_byte(uint8_t c) {
        set_txen();
        write(c);
    }

    void setup() {
        rs485slave.begin();
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