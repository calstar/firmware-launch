#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "mbed.h"
#include "pins.h"

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (250000u) // 250*1000us = 250ms
#define BUF_SIZE (256)

using namespace flatbuffers;
using namespace Calstar;

const UplinkMsg *getUplinkMsg(char c);
const FCUpdateMsg *getFCUpdateMsg(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
// FCUpdateMsg fcLatestData;
// bool fcPowered = false;

DigitalOut fcPower(FC_SWITCH);

Serial rs422(RS422_TX, RS422_RX);
Serial debug_uart(DEBUG_TX, DEBUG_RX);

us_timestamp_t last_msg_send_us;

void start() {
    fcPower = 0;

    msgTimer.start();

    rs422.baud(RS422_BAUDRATE);
    rs422.set_blocking(false);

    debug_uart.baud(DEBUG_UART_BAUDRATE);
    debug_uart.set_blocking(false);
    debug_uart.printf("---- CalSTAR Telemetry/Power Control ----\r\n");

    last_msg_send_us = msgTimer.read_high_resolution_us();
}

void loop() {
    // Send a message every MSG_SEND_INTERVAL_US microseconds
    us_timestamp_t current_time = msgTimer.read_high_resolution_us();
    if (current_time >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
        buildCurrentMessage();

        debug_uart.printf("Sending downlink with fcPowered=%d\r\n", (int)fcPower);
        // TODO: send over radio

        last_msg_send_us = current_time;
    }
    // Always read messages
    if (rs422.readable()) {
        // Read into message type
        const FCUpdateMsg *msg = getFCUpdateMsg(rs422.getc());
        if (msg) {
            // fcLatestData = msg;
        }
    }

    if (debug_uart.readable()) {
        char c = debug_uart.getc();
        if (c == 'p') {
            // toggle FC power
            // fcPowered ^= true;
            // fcPower = fcPowered ? 1 : 0;
            fcPower = 1 - fcPower;
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
    static uint8_t return_buffer[BUF_SIZE];
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
        memcpy(return_buffer, buffer, len);
        const UplinkMsg *output = GetUplinkMsg(return_buffer);
        // The message knows how big it should be
        uint8_t expectedBytes = output->Bytes();

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
                actual_len = expectedBytes;
                // If it is a message, then make the return buffer just hold it (clear the extra bytes we copied)
                memset(return_buffer+actual_len, 0, len - actual_len);
            } else {
                // If it isn't valid, then this buffer just has some malformed messages... continue and let's get
                // them out of the buffer by reading more
                return NULL;
            }
        }

        // Now that we've read a valid message, remove it from the buffer and move everything else down
        // Then reduce current buffer length by the length of the processed message
        // Then clear the rest of the buffer so that we don't get false positives with the verifiers
        memmove(buffer, buffer+actual_len, BUF_SIZE-actual_len);
        len -= actual_len;
        memset(buffer+len, 0, BUF_SIZE-len);

        return output;
    }
    return NULL;
}
const FCUpdateMsg *getFCUpdateMsg(char c) {
    static uint8_t buffer[BUF_SIZE];
    static uint8_t return_buffer[BUF_SIZE];
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
        memcpy(return_buffer, buffer, len);
        const FCUpdateMsg *output = GetFCUpdateMsg(return_buffer);
        // The message knows how big it should be
        uint8_t expectedBytes = output->Bytes();

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
                actual_len = expectedBytes;
                // If it is a message, then make the return buffer just hold it (clear the extra bytes we copied)
                memset(return_buffer+actual_len, 0, len - actual_len);
            } else {
                // If it isn't valid, then this buffer just has some malformed messages... continue and let's get
                // them out of the buffer by reading more
                return NULL;
            }
        }

        // Now that we've read a valid message, remove it from the buffer and move everything else down
        // Then reduce current buffer length by the length of the processed message
        // Then clear the rest of the buffer so that we don't get false positives with the verifiers
        memmove(buffer, buffer+actual_len, BUF_SIZE-actual_len);
        len -= actual_len;
        memset(buffer+len, 0, BUF_SIZE-len);

        return output;
    }
    return NULL;
}

void buildCurrentMessage() {
    builder.Reset();
    Offset<FCUpdateMsg> fcUpdateMsg = CreateFCUpdateMsg(builder,
        1, // Can't be 0 or it will be ignored
        FCState_Pad,
        0.0f, 1.0f, 2.0f,
        3.0f, 4.0f, 5.0f,
        6.0f, 7.0f, 8.0f,
        9.0f, 10.0f,
        false, false,
        false, true,
        false, false,
        false, true,
        false, false,
        false, true,
        false, false);
    Offset<DownlinkMsg> message = CreateDownlinkMsg(builder,
        1, // Can't be 0 or it will be ignored
        TPCState_Pad,
        (int)fcPower,
        fcUpdateMsg, // TODO: use the latest FC data
        builder.CreateString("gps test"),
        123);
    builder.Finish(message);

    uint8_t bytes = (uint8_t)builder.GetSize();
    builder.Reset();
    fcUpdateMsg = CreateFCUpdateMsg(builder,
        1, // TODO: does the byte count here matter, since it's a sub-message?
        FCState_Pad,
        0.0f, 1.0f, 2.0f,
        3.0f, 4.0f, 5.0f,
        6.0f, 7.0f, 8.0f,
        9.0f, 10.0f,
        false, false,
        false, true,
        false, false,
        false, true,
        false, false,
        false, true,
        false, false);
    message = CreateDownlinkMsg(builder,
        bytes, // Fill in actual number of bytes
        TPCState_Pad,
        (int)fcPower,
        fcUpdateMsg, // TODO: use the latest FC data
        builder.CreateString("gps test"),
        123);
    builder.Finish(message);
}
