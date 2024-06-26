#ifdef DCSBIOS_ESP32_ID

#include <pb_decode.h>
#include "SlaveMessage.pb.h"

#ifndef DCSBIOS_ESP32_WIFI_SSID
#ifdef DCSBIOS_ESP32_WIZNET
	#include <Ethernet_Generic.h> // khoih-prog/Ethernet_Generic@^2.8.1
#endif
#ifdef DCSBIOS_ESP32_ENC28J60
	#include <Ethernet.h> // jandrassy/EthernetENC@^2.0.4
#endif
#endif

namespace DcsBios {
	void ESP32NetworkedSlave::begin() {
		beginNeopixel();
		Serial.begin(115200);
		setNeopixel(OFFLINE);
		
		// Connect to WiFi
		if (!connect_phy()) {
			Serial.println("Restarting.");
			ESP.restart();
		}

		// Load a protocol client based on #defines.
		set_client();
	}

	bool ESP32NetworkedSlave::connect_phy() {
		#ifdef DCSBIOS_ESP32_WIFI_SSID
			return connect_wifi();
		#else
			return connect_ethernet();
		#endif
	}

	bool ESP32NetworkedSlave::connect_wifi() {
		#if defined(DCSBIOS_ESP32_WIFI_SSID) && defined(DCSBIOS_ESP32_WIFI_SSID)
		WiFi.begin(DCSBIOS_ESP32_WIFI_SSID, DCSBIOS_ESP32_WIFI_PASSWORD);
		#endif

		unsigned long start_time = millis();
		
		while (WiFi.status() != WL_CONNECTED) {
			Serial.println("Connecting to WiFi...");
			
			// Check association timeout, sometimes restarting the ESP32 is faster than waiting.
			if (millis() - start_time > DCSBIOS_ESP32_ASSOCIATION_TIMEOUT) {
				Serial.println("WiFi association timed out.");

				return false;
			}

			delay(500);
		}

		setNeopixel(ASSOCIATED);
		Serial.println("Connected to " + WiFi.SSID() + " with IP " + WiFi.localIP().toString());

		return true;
	}

	bool ESP32NetworkedSlave::connect_ethernet() {
		#ifndef DCSBIOS_ESP32_WIFI_SSID
		Ethernet.init(DCSBIOS_ESP32_ETHERNET_CS);
		Ethernet.setHostname((String("dcs-") + DCSBIOS_ESP32_ID).c_str());
		
		Serial.println("Initialize Ethernet with DHCP:");
		
		uint8_t mac[6];
    	generateMACAddress(DCSBIOS_ESP32_ID, mac); // Generate a stable MAC address based on the device ID.
		
		if (Ethernet.begin(mac) == 0) {
			Serial.println("Failed to configure Ethernet using DHCP");
			// Check for Ethernet hardware present
			if (Ethernet.hardwareStatus() == EthernetNoHardware) {
				Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
				while (true) {
					delay(1); // do nothing, no point running without Ethernet hardware
				}
			}
			if (Ethernet.linkStatus() == LinkOFF) {
				Serial.println("Ethernet cable is not connected.");
			}

			return false;
		} else {
			Serial.print("  DHCP assigned IP ");
			Serial.println(Ethernet.localIP());
		}
		// give the Ethernet shield a second to initialize:
		delay(1000);

		#endif

		return true;
	}

	void ESP32NetworkedSlave::generateMACAddress(const std::string& identifier, uint8_t mac[6]) {
		// Simple hash: XOR and rotate
		uint32_t hash = 0;
		for (char c : identifier) {
			hash = (hash << 1) | (hash >> (31)); // Rotate left
			hash ^= c; // XOR with the character
		}

		// Use parts of the hash to fill in the MAC address
		mac[0] = 0x02; // Locally administered and unicast
		mac[1] = (hash >> 24) & 0xFF;
		mac[2] = (hash >> 16) & 0xFF;
		mac[3] = (hash >> 8) & 0xFF;
		mac[4] = hash & 0xFF;
		mac[5] = (hash >> 4) & 0xFF;

		Serial.print("Generated MAC address: ");
		for (int i = 0; i < 6; ++i) {
			if(mac[i] < 16) Serial.print("0");
			Serial.print(mac[i], HEX);
			if (i < 5) Serial.print(":");
		}
		Serial.println();
	}

	bool ESP32NetworkedSlave::disconnect_phy() {
		setNeopixel(OFFLINE);

		#ifdef DCSBIOS_ESP32_WIFI_SSID
			return disconnect_wifi();
		#else
			return disconnect_ethernet();
		#endif
	}

	bool ESP32NetworkedSlave::disconnect_wifi() {
		return WiFi.disconnect(true, true);
	}

	bool ESP32NetworkedSlave::disconnect_ethernet() {
		#ifdef DCSBIOS_ESP32_WIZNET
		Ethernet.softreset();
		delay(100);
		Ethernet.hardreset();
		#endif

		return true;
	}

	void ESP32NetworkedSlave::set_client() {
		#ifdef DCSBIOS_ESP32_MULTICAST
		client = new DcsBiosESP32MulticastClient(receive_queue);
		#elif defined(DCSBIOS_ESP32_TCP)
		client = new DcsBiosESP32TCPClient(receive_queue);
		#elif defined(DCSBIOS_ESP32_UDP)
		client = new DcsBiosESP32UnicastClient(receive_queue);
		#else
		#error "Communication protocol not selected. Please #define either DCSBIOS_ESP32_MULTICAST, DCSBIOS_ESP32_UDP or DCSBIOS_ESP32_TCP."
		#endif
	}

	void ESP32NetworkedSlave::loop() {
		#ifdef DCSBIOS_ESP32_INSTRUMENTATION
		lastLoopDuration = micros() - lastLoopTime;
		lastLoopTime = micros();
		#endif

		client->loop();

		if (!client->connected()) { // If not connected, ask the client to discover the Nexus.
			if (client->discover()) {
				if (client->start()) {
					// Reset timeout when connected
					lastReceivedTime = millis();
					lastKeepAliveTime = millis();

					setNeopixel(CONNECTED);
				}
			} else {
				Serial.println("No services found.");

				delay(250);
			}
		} else { // If connected
			// Check if there are messages to process
			receive_queue.for_each_remove_if([&](Message& message) -> bool {
				lastReceivedTime = millis();

				if (message.type == "message") { // This is a DCS-BIOS message that needs to be forwarded to the parser
					for (unsigned char c : message.data) {
						parser.processChar(c);
					}
				} else if (message.type == "check-in") { // Nexus is checking in, nothing to do. lastReceivedTime is automatically updated at the top of this function.
					Serial.println("Received check-in");
				} else if (message.type == "ack") { // Nexus has acknowledged a message, remove it from the send_queue.
					Serial.println("Received ack for " + String(message.seq));
					send_queue.for_each_remove_if([&](Message& queued_packet) -> bool {
						if (message.slave_details.id == DCSBIOS_ESP32_ID && queued_packet.seq == message.seq) {
							return true; // Remove this packet from send_queue
						}
						return false; // Do not remove this packet from send_queue
					});

				} else if ((message.type == "restart" && message.slave_details.id == DCSBIOS_ESP32_ID) || message.type == "restart-all") { // Restart command.
					Serial.println("Restarting on Nexus request.");
					ESP.restart();
				}

				return true;
			});

			// Check for time-out.
			if (millis() - lastReceivedTime > DCSBIOS_ESP32_RECEIVE_TIMEOUT) {
				client->stop();
				setNeopixel(ASSOCIATED);
				Serial.println("Connection timed out, restarting.");
			}

			// Send check-in
			if (millis() - lastKeepAliveTime >= DCSBIOS_ESP32_KEEPALIVE_PERIOD) {
				enqueue("check-in", true);

				lastKeepAliveTime = millis();
			}
			
			// Send enqueued data
			send_queue.for_each_remove_if([&](Message& message) -> bool {
				if (millis() - message.lastSentTime > DCSBIOS_ESP32_SEND_RETRY_DELAY) {
					client->send(message, message.force);
					message.retries++;
					message.lastSentTime = millis();
				}

				if (message.retries >= DCSBIOS_ESP32_SEND_RETRIES) {
					// Serial.println("Reached maximum retries for " + String(message.seq) + " after " + String(message.retries) + " retries.");
					
					return true; // Remove this message from send_queue
				} 

				return false; // Do not remove this message from send_queue
			});
		}
	}

	void ESP32NetworkedSlave::enqueue(const char* type, bool force) {
		enqueue(type, "", force);
	}

	void ESP32NetworkedSlave::enqueue(const char* type, String data, bool force) {
		Message message(type, data, ++last_message_id);

		message.force = force;
		message.slave_details.id = DCSBIOS_ESP32_ID;
		message.slave_details.mac = WiFi.macAddress();

		if (message.type == "check-in") {
			message.slave_details.rssi = WiFi.RSSI();
			message.slave_details.loop_duration = lastLoopDuration;
			message.slave_details.free_heap = ESP.getFreeHeap();
			message.slave_details.cpu_freq = ESP.getCpuFreqMHz();
			message.slave_details.flash_size = ESP.getFlashChipSize();
			message.retries = DCSBIOS_ESP32_SEND_RETRIES; // Check-in messages are not important, so we set retries to the maximum to prevent them from being retried.
		}

		send_queue.push_back(message);

		Serial.println("Enqueued " + String(type) + " with id " + String(message.seq));
	}

	// Standard DCS-BIOS functions
	void setup() {
		wifiSlave.begin();
	}

	void loop() {
		wifiSlave.loop();
		PollingInput::pollInputs();
		ExportStreamListener::loopAll();
	}

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		String data = msg + String(' ') + arg + String('\n');
		wifiSlave.enqueue("message", data);
		
		DcsBios::PollingInput::setMessageSentOrQueued();

		return true;
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}
}
#endif
