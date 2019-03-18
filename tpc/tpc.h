#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "mbed.h"
#include "pins.h"
#include "RFM69/RFM69.hpp"

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (250000u) // 250*1000us = 250ms
#define BUF_SIZE (256)
// random 16 bytes that must be the same across all nodes
#define ENCRYPT_KEY ("CALSTARENCRYPTKE")

using namespace flatbuffers;
using namespace Calstar;

const UplinkMsg *getUplinkMsg(char c);
const FCUpdateMsg *getFCUpdateMsg(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
const FCUpdateMsg *fcLatestData = NULL;

DigitalOut fcPower(FC_SWITCH);

Serial rs422(RS422_TX, RS422_RX);
Serial debug_uart(DEBUG_TX, DEBUG_RX);

us_timestamp_t last_msg_send_us;

RFM69 radio(SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_SSEL, RADIO_RST, true);

uint8_t fcUpdateMsgBuffer[BUF_SIZE];
uint8_t uplinkMsgBuffer[BUF_SIZE];

void start() {
    fcPower = 0;

    msgTimer.start();

    rs422.baud(RS422_BAUDRATE);
    rs422.set_blocking(false);

    debug_uart.baud(DEBUG_UART_BAUDRATE);
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
        
        const DownlinkMsg *msg = GetDownlinkMsg(builder.GetBufferPointer());
        if (msg->FCMsg() && msg->FCMsg()->Altitude() != 9.0f) {
            debug_uart.printf("[Bytes=%d,State=%d,FCPowered=%d,FCMsg={",
                (int)msg->Bytes(), (int)msg->State(), (int)msg->FCPowered());
            if (msg->FCMsg()) {
                debug_uart.printf("Bytes=%d,State=%d,Accel=<%f,%f,%f>,Mag=<%f,%f,%f>,Gyro=<%f,%f,%f>,Alt=%f,Pressure=%f,BP1=<%d,%d>,BP2=<%d,%d>,BP3=<%d,%d>,BP4=<%d,%d>,BP5=<%d,%d>,BP6=<%d,%d>,BP7=<%d,%d>",
                    (int)msg->FCMsg()->Bytes(),
                    (int)msg->FCMsg()->State(),
                    msg->FCMsg()->AccelX(),
                    msg->FCMsg()->AccelY(),
                    msg->FCMsg()->AccelZ(),
                    msg->FCMsg()->MagX(),
                    msg->FCMsg()->MagY(),
                    msg->FCMsg()->MagZ(),
                    msg->FCMsg()->GyroX(),
                    msg->FCMsg()->GyroY(),
                    msg->FCMsg()->GyroZ(),
                    msg->FCMsg()->Altitude(),
                    msg->FCMsg()->Pressure(),
                    (int)msg->FCMsg()->BP1Continuity(),
                    (int)msg->FCMsg()->BP1Ignited(),
                    (int)msg->FCMsg()->BP2Continuity(),
                    (int)msg->FCMsg()->BP2Ignited(),
                    (int)msg->FCMsg()->BP3Continuity(),
                    (int)msg->FCMsg()->BP3Ignited(),
                    (int)msg->FCMsg()->BP4Continuity(),
                    (int)msg->FCMsg()->BP4Ignited(),
                    (int)msg->FCMsg()->BP5Continuity(),
                    (int)msg->FCMsg()->BP5Ignited(),
                    (int)msg->FCMsg()->BP6Continuity(),
                    (int)msg->FCMsg()->BP6Ignited(),
                    (int)msg->FCMsg()->BP7Continuity(),
                    (int)msg->FCMsg()->BP7Ignited());
            }
            debug_uart.printf("},GPS=\"%s\",Batt=%d,Frame=%d,AckReqd=%d,Time=%llu,Type=%d]\r\n",
                msg->GPSString()->str().c_str(),
                (int)msg->BattVoltage(),
                (int)msg->FrameID(),
                (int)msg->AckReqd(),
                msg->TimeStamp(),
                (int)msg->Type());
        }

        // Send over radio
        uint8_t *buf = builder.GetBufferPointer();
        int size = builder.GetSize();
        radio.send(buf, size);

        last_msg_send_us = current_time;
    }
    // Always read messages
    while (rs422.readable()) {
        // Read into message type
        const FCUpdateMsg *msg = getFCUpdateMsg(rs422.getc());
        if (msg) {
            if (msg->Altitude() != 9.0f) {
                debug_uart.printf("Read FC update message (B=%d,S=%d,A=<%f,%f,%f>,M=<%f,%f,%f>,G=<%f,%f,%f>,A=%f,P=%f,BP=<%d,%d,%d,%d,%d,%d,%d>).\r\n",
                    (int)msg->Bytes(),
                    (int)msg->State(),
                    msg->AccelX(),
                    msg->AccelY(),
                    msg->AccelZ(),
                    msg->MagX(),
                    msg->MagY(),
                    msg->MagZ(),
                    msg->GyroX(),
                    msg->GyroY(),
                    msg->GyroZ(),
                    msg->Altitude(),
                    msg->Pressure(),
                    msg->BP1Ignited(),
                    msg->BP2Ignited(),
                    msg->BP3Ignited(),
                    msg->BP4Ignited(),
                    msg->BP5Ignited(),
                    msg->BP6Ignited(),
                    msg->BP7Ignited());
            }
            fcLatestData = msg;
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
                rs422.write(uplinkMsgBuffer, msg->Bytes(), NULL);
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
        builder.CreateString("gps test"),
        123,
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
        builder.CreateString("gps test"),
        123,
        0,
        false,
        currentTime,
        DownlinkType_StateUpdate);
    builder.Finish(message);

    fcLatestData = NULL;
}
