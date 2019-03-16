#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "mbed.h"
#include "pins.h"

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (200000u) // 200*1000us = 200ms
#define BUF_SIZE (1024)

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
    static unsigned int len = 0;
    // static unsigned int messageSize = 0;

    if (len == BUF_SIZE) {
        len = 0;
    }
    buffer[len++] = (uint8_t)c;
    Verifier v(buffer, len);
    if (VerifyUplinkMsgBuffer(v)) {
        len = 0;
        return GetUplinkMsg(buffer);
    }
    return NULL;
}
const FCUpdateMsg *getFCUpdateMsg(char c) {
    static uint8_t buffer[BUF_SIZE];
    static unsigned int len = 0;

    if (len == BUF_SIZE) {
        len = 0;
    }
    buffer[len++] = (uint8_t)c;
    Verifier v(buffer, len);
    if (VerifyFCUpdateMsgBuffer(v)) {
        len = 0;
        return GetFCUpdateMsg(buffer);
    }
    return NULL;
}

void buildCurrentMessage() {
    Offset<FCUpdateMsg> fcUpdateMsg = CreateFCUpdateMsg(builder,
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
        TPCState_Pad,
        (int)fcPower,
        fcUpdateMsg, // TODO: use the latest FC data
        builder.CreateString("gps test"),
        123);
    builder.Finish(message);
}
