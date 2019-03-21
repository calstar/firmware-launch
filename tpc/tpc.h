#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "mbed.h"
#include "pins.h"
#include "RFM69/RFM69.hpp"
#include <string>

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define GPS_BAUDRATE (9600) // Must be 9600
#define MSG_SEND_INTERVAL_US (250000u) // 250*1000us = 250ms
#define BUF_SIZE (256)
// random 16 bytes that must be the same across all nodes
#define ENCRYPT_KEY ("CALSTARENCRYPTKE")

// analog in is 0-1 from gnd to VCC (3.3V)
// so multiply by 3.3V first, then multiply by inverse of resistive divider
// (92.2kOhm / 502.2kOhm)
// then we have a recalibration factor added at the end
#define AIN_TO_VOLTAGE (3.3f * 502.2f / 92.2f * 1.10201042442293f)

using namespace flatbuffers;
using namespace Calstar;

const UplinkMsg *getUplinkMsg(char c);
const FCUpdateMsg *getFCUpdateMsg(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
const FCUpdateMsg *fcLatestData = NULL;

DigitalOut fcPower(FC_SWITCH);
AnalogIn battVoltage(BATT_VOLTAGE);

UARTSerial rs422(RS422_TX, RS422_RX, RS422_BAUDRATE);
UARTSerial gps_uart(GPS_TX, GPS_RX, GPS_BAUDRATE);
Serial debug_uart(DEBUG_TX, DEBUG_RX, DEBUG_UART_BAUDRATE);

us_timestamp_t last_msg_send_us;

RFM69 radio(SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_SSEL, RADIO_RST, true);

uint8_t fcUpdateMsgBuffer[BUF_SIZE];
uint8_t uplinkMsgBuffer[BUF_SIZE];

uint8_t rs422_read_buf[BUF_SIZE];
uint8_t gps_read_buf[BUF_SIZE];
std::string gps_sentence_builder;
std::string gpsLatestData;

void start() {
    fcPower = 0;

    msgTimer.start();

    // rs422.set_blocking(true);

    debug_uart.set_blocking(false);
    debug_uart.printf("---- CalSTAR Telemetry/Power Control ----\r\n");

    last_msg_send_us = msgTimer.read_high_resolution_us();

    
    radio.reset();
    debug_uart.printf("Radio reset complete.\r\n");

    radio.init();
    radio.setAESEncryption(ENCRYPT_KEY, strlen(ENCRYPT_KEY));

    radio.setHighPowerSettings(true);
    radio.setPowerDBm(20);

    debug_uart.printf("Radio init complete.\r\n");
}

void loop() {
    // Send a message every MSG_SEND_INTERVAL_US microseconds
    us_timestamp_t current_time = msgTimer.read_high_resolution_us();
    if (current_time >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
        buildCurrentMessage();

        debug_uart.printf("Sending radio downlink with fcPowered=%d [%d bytes].\r\n",
            (int)fcPower,
            (int)builder.GetSize());

        // Send over radio
        uint8_t *buf = builder.GetBufferPointer();
        int size = builder.GetSize();
        radio.send(buf, size);

        last_msg_send_us = current_time;
    }
    // Always read messages
    while (rs422.readable()) {
        ssize_t num_read = rs422.read(rs422_read_buf, BUF_SIZE);
        for (int i = 0; i < num_read; i++) {
            const FCUpdateMsg *msg = getFCUpdateMsg(rs422_read_buf[i]);
            if (msg) {
                fcLatestData = msg;
            }
        }
    }

    while (debug_uart.readable()) {
        char c = debug_uart.getc();
        if (c == 'p') {
            // toggle FC power
            fcPower = 1 - fcPower;
            if (fcPower) {
                debug_uart.printf("Turned on FC power.\r\n");
            } else {
                debug_uart.printf("Turned off FC power.\r\n");
            }
        }
    }

    while (gps_uart.readable()) {
        ssize_t num_read = gps_uart.read(gps_read_buf, BUF_SIZE);
        for (int i = 0; i < num_read; i++) {
            gps_sentence_builder += (char)gps_read_buf[i];
        }

        size_t crlf_pos = gps_sentence_builder.find("\r\n");
        if (crlf_pos != std::string::npos) {
            gpsLatestData = gps_sentence_builder.substr(0, crlf_pos);
            gps_sentence_builder = gps_sentence_builder.substr(crlf_pos + 2); // skip "\r\n"
        }
    }

    static char radio_rx_buf[256];
    int num_bytes_received = radio.receive(radio_rx_buf, sizeof(radio_rx_buf));
    // We skip the first byte; everything else is valid.
    if (num_bytes_received > 1) {
        debug_uart.printf("Received %d bytes from radio [RSSI=%d].\r\n",
            num_bytes_received - 1, radio.getRSSI());
    }
    for (int i = 1; i < num_bytes_received; i++) {
        const UplinkMsg *msg = getUplinkMsg(radio_rx_buf[i]);
        if (msg) {
            // Received an uplink over radio -- FCOn/FCOff/Ack is for us, BlackPowderPulse is for FC
            debug_uart.printf("--> Received uplink message -->\r\n");
            if (msg->Type() == UplinkType_FCOn) {
                fcPower = 1;
                debug_uart.printf("    Turned on FC power.\r\n");
            } else if (msg->Type() == UplinkType_FCOff) {
                fcPower = 0;
                debug_uart.printf("    Turned off FC power.\r\n");
            } else if (msg->Type() == UplinkType_Ack) {
                // TODO: deal with acks
                debug_uart.printf("    Acking message receive (TODO).\r\n");
            } else {
                // For other messages we just forward to FC.
                // msg came from uplinkMsgBuffer, so we just use the buffer here
                debug_uart.printf("    Forwarded BlackPowderPulse(");
                for (uoffset_t i = 0; i < msg->BP()->size(); i++) {
                    debug_uart.printf("%d", msg->BP()->Get(i));
                    if (i != msg->BP()->size()-1) {
                        debug_uart.printf(",");
                    }
                }
                debug_uart.printf(")to FC.\r\n");
                rs422.write(uplinkMsgBuffer, msg->Bytes());
            }
        }
    }
}

int main() {
  start();
  while (1) {
    loop();
  }
}

const UplinkMsg *getUplinkMsg(char c) {
    static uint8_t buffer[BUF_SIZE];
    static unsigned int len = 0;

    if (len == BUF_SIZE) {
        // If at end of buffer, shift and add to end
        memmove(buffer, buffer + 1, BUF_SIZE-1);
        buffer[BUF_SIZE-1] = (uint8_t)c;
    } else {
        // Otherwise build up buffer
        buffer[len++] = (uint8_t)c;
    }

    // The verifier will say that buf has a valid message for any length from actual_length-some number to full buffer length
    // So basically, we trust the verifier, but verify separately by having a #-bytes field in the message itself
    // So if the verifier says there's a valid message in the buffer, we read that message, get the number of bytes that the
    // message says it should be, and actually process a message of THAT size.
    Verifier verifier(buffer, len);
    if (VerifyUplinkMsgBuffer(verifier)) {
        const UplinkMsg *msg = GetUplinkMsg(buffer);
        // The message knows how big it should be
        uint8_t expectedBytes = msg->Bytes();

        uint8_t actual_len = len;
        if (len < expectedBytes) {
            // The verifier will say we have a valid message even if we're a few bytes short
            // Just read more characters at this point by returning early
            return NULL;
        } else if (len > expectedBytes) {
            // Now we want to verify that the "smaller buffer" with length equal to the expected number of bytes
            // is actually a message in its own right (just a double check basically)
            Verifier smallerVerifier(buffer, expectedBytes);
            if (VerifyUplinkMsgBuffer(smallerVerifier)) {
                // If it is a message, then make sure we use the correct (smaller) length
                actual_len = expectedBytes;
            } else {
                // If it isn't valid, then this buffer just has some malformed messages... continue and let's get
                // them out of the buffer by reading more
                return NULL;
            }
        }

        // Now that we've read a valid message, copy it into the output buffer,
        // then remove it from the input buffer and move everything else down.
        // Then reduce current buffer length by the length of the processed message
        // Then clear the rest of the buffer so that we don't get false positives with the verifiers
        memcpy(uplinkMsgBuffer, buffer, actual_len);
        memmove(buffer, buffer+actual_len, BUF_SIZE-actual_len);
        len -= actual_len;
        // Clear the rest of the buffer
        memset(buffer+len, 0, BUF_SIZE-len);

        return GetUplinkMsg(uplinkMsgBuffer);
    }
    return NULL;
}
const FCUpdateMsg *getFCUpdateMsg(char c) {
    static uint8_t buffer[BUF_SIZE];
    static unsigned int len = 0;

    if (len == BUF_SIZE) {
        // If at end of buffer, shift and add to end
        memmove(buffer, buffer + 1, BUF_SIZE-1);
        buffer[BUF_SIZE-1] = (uint8_t)c;
    } else {
        // Otherwise build up buffer
        buffer[len++] = (uint8_t)c;
    }

    // The verifier will say that buf has a valid message for any length from actual_length-some number to full buffer length
    // So basically, we trust the verifier, but verify separately by having a #-bytes field in the message itself
    // So if the verifier says there's a valid message in the buffer, we read that message, get the number of bytes that the
    // message says it should be, and actually process a message of THAT size.
    Verifier verifier(buffer, len);
    if (VerifyFCUpdateMsgBuffer(verifier)) {
        const FCUpdateMsg *msg = GetFCUpdateMsg(buffer);
        // The message knows how big it should be
        uint8_t expectedBytes = msg->Bytes();

        uint8_t actual_len = len;
        if (len < expectedBytes) {
            // The verifier will say we have a valid message even if we're a few bytes short
            // Just read more characters at this point by returning early
            return NULL;
        } else if (len > expectedBytes) {
            // Now we want to verify that the "smaller buffer" with length equal to the expected number of bytes
            // is actually a message in its own right (just a double check basically)
            Verifier smallerVerifier(buffer, expectedBytes);
            if (VerifyFCUpdateMsgBuffer(smallerVerifier)) {
                // If it is a message, then make sure we use the correct (smaller) length
                actual_len = expectedBytes;
            } else {
                // If it isn't valid, then this buffer just has some malformed messages... continue and let's get
                // them out of the buffer by reading more
                return NULL;
            }
        }

        // Now that we've read a valid message, copy it into the output buffer,
        // then remove it from the input buffer and move everything else down.
        // Then reduce current buffer length by the length of the processed message
        // Then clear the rest of the buffer so that we don't get false positives with the verifiers
        memcpy(fcUpdateMsgBuffer, buffer, actual_len);
        memmove(buffer, buffer+actual_len, BUF_SIZE-actual_len);
        len -= actual_len;
        // Clear the rest of the buffer
        memset(buffer+len, 0, BUF_SIZE-len);

        return GetFCUpdateMsg(fcUpdateMsgBuffer);
    }
    return NULL;
}

void buildCurrentMessage() {
    us_timestamp_t currentTime = msgTimer.read_high_resolution_us();
    uint16_t battVoltage_uint = battVoltage.read_u16();

    builder.Reset();
    Offset<FCUpdateMsg> fcUpdateMsg;
    if (fcLatestData) {
        fcUpdateMsg = CreateFCUpdateMsg(builder,
            fcLatestData->Bytes(),
            fcLatestData->State(),
            fcLatestData->AccelX(), fcLatestData->AccelY(), fcLatestData->AccelZ(),
            fcLatestData->MagX(), fcLatestData->MagY(), fcLatestData->MagZ(),
            fcLatestData->GyroX(), fcLatestData->GyroY(), fcLatestData->GyroZ(),
            fcLatestData->Altitude(), fcLatestData->Pressure(),
            fcLatestData->BP1Continuity(), fcLatestData->BP1Ignited(),
            fcLatestData->BP2Continuity(), fcLatestData->BP2Ignited(),
            fcLatestData->BP3Continuity(), fcLatestData->BP3Ignited(),
            fcLatestData->BP4Continuity(), fcLatestData->BP4Ignited(),
            fcLatestData->BP5Continuity(), fcLatestData->BP5Ignited(),
            fcLatestData->BP6Continuity(), fcLatestData->BP6Ignited(),
            fcLatestData->BP7Continuity(), fcLatestData->BP7Ignited());
    } else {
        // Null it if we don't have any FC data yet
        fcUpdateMsg = 0;
    }
    Offset<DownlinkMsg> message = CreateDownlinkMsg(builder,
        1, // Can't be 0 or it will be ignored
        TPCState_Pad,
        (int)fcPower,
        fcUpdateMsg,
        builder.CreateString(gpsLatestData),
        battVoltage_uint,
        0,
        false,
        currentTime,
        DownlinkType_StateUpdate);
    builder.Finish(message);

    uint8_t bytes = (uint8_t)builder.GetSize();
    builder.Reset();
    if (fcLatestData) {
        fcUpdateMsg = CreateFCUpdateMsg(builder,
            fcLatestData->Bytes(),
            fcLatestData->State(),
            fcLatestData->AccelX(), fcLatestData->AccelY(), fcLatestData->AccelZ(),
            fcLatestData->MagX(), fcLatestData->MagY(), fcLatestData->MagZ(),
            fcLatestData->GyroX(), fcLatestData->GyroY(), fcLatestData->GyroZ(),
            fcLatestData->Altitude(), fcLatestData->Pressure(),
            fcLatestData->BP1Continuity(), fcLatestData->BP1Ignited(),
            fcLatestData->BP2Continuity(), fcLatestData->BP2Ignited(),
            fcLatestData->BP3Continuity(), fcLatestData->BP3Ignited(),
            fcLatestData->BP4Continuity(), fcLatestData->BP4Ignited(),
            fcLatestData->BP5Continuity(), fcLatestData->BP5Ignited(),
            fcLatestData->BP6Continuity(), fcLatestData->BP6Ignited(),
            fcLatestData->BP7Continuity(), fcLatestData->BP7Ignited());
    } else {
        // Null it if we don't have any FC data yet
        fcUpdateMsg = 0;
    }
    message = CreateDownlinkMsg(builder,
        bytes, // Fill in actual number of bytes
        TPCState_Pad,
        (int)fcPower,
        fcUpdateMsg,
        builder.CreateString(gpsLatestData),
        battVoltage_uint,
        0,
        false,
        currentTime,
        DownlinkType_StateUpdate);
    builder.Finish(message);

    fcLatestData = NULL;
}
