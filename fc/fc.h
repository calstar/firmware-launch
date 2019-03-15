#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "mbed.h"
#include "pins.h"

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (100000u) // 100*1000us = 100ms
#define NUM_BP (7)
#define BUF_SIZE (1024)

using namespace flatbuffers;
using namespace Calstar;

const UplinkMsg *getUplinkMsg(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
bool bpIgnited[NUM_BP];


DigitalOut led_red(STATE_LED_RED);
DigitalOut led_green(STATE_LED_GREEN);

Serial rs422(RS422_TX, RS422_RX);
Serial debug_uart(DEBUG_TX, DEBUG_RX);

us_timestamp_t last_msg_send_us;

void loop() {
    // Send a message every MSG_SEND_INTERVAL_US microseconds
    us_timestamp_t current_time = msgTimer.read_high_resolution_us();
    if (current_time >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
        uint8_t* buf = builder.GetBufferPointer();
        int size = builder.GetSize();

        // Write the size of the message
        // void* size_ptr = (void*)(&size);
        // rs422.write((uint8_t*)size_ptr, sizeof(size), NULL);
        // then write the message
        rs422.write(buf, size, NULL);

        // also just toggle the red LED at this point
        led_red = (led_red == 0) ? 1 : 0;

        debug_uart.printf("Sending FC update\r\n");

        last_msg_send_us = current_time;
    }
    // Always read messages
    if (rs422.readable()) {
        // Read into message type
        const UplinkMsg *msg = getUplinkMsg(rs422.getc());
        if (msg) {
            UplinkType type = msg->Type();

            // If turning on/off black powder, for now we just set our state
            // (in the future we will actually do black powder stuff)
            if (type == UplinkType_BlackPowderOn || type == UplinkType_BlackPowderOff) {
                const Vector<uint8_t> *bp = msg->BP();
                for (int i = 0; i < NUM_BP; i++) {
                    if (bp->Get(i)) {
                        bpIgnited[i] = (type == UplinkType_BlackPowderOn);
                    }
                }

                // Update the message being sent out to T/PC
                buildCurrentMessage();
            }
        }
    }

    if (debug_uart.readable()) {
        char c = debug_uart.getc();

        if (c == 'r') {
            led_red = (led_red == 0) ? 1 : 0;
        } else if (c == 'g') {
            led_green = (led_green == 0) ? 1 : 0;
        }
    }
}

void start() {
    led_red = 0;
    led_green = 1;

    msgTimer.start();

    rs422.baud(RS422_BAUDRATE);
    rs422.set_blocking(false);

    debug_uart.baud(DEBUG_UART_BAUDRATE);
    debug_uart.set_blocking(false);
    debug_uart.printf("---- CalSTAR Flight Computer ----\r\n");

    buildCurrentMessage();

    last_msg_send_us = msgTimer.read_high_resolution_us();
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
    // if (pos == sizeof(int)) {
    //     // Read the size of the incoming message
    //     void *buf = buffer;
    //     messageSize = *((int*)buf);
    // } else if (pos == sizeof(int) + messageSize) {
    //     // Reached end of message
    //     void *result = buffer + sizeof(int);
    //     pos = 0;
    //     messageSize = 0;
    //     return result;
    // }
    //
    // return NULL;
}

void buildCurrentMessage() {
    Offset<FCUpdateMsg> message = CreateFCUpdateMsg(builder,
        FCState_Pad,
        0.0f, 1.0f, 2.0f,
        3.0f, 4.0f, 5.0f,
        6.0f, 7.0f, 8.0f,
        9.0f, 10.0f,
        bpIgnited[0], false,
        bpIgnited[1], true,
        bpIgnited[2], false,
        bpIgnited[3], true,
        bpIgnited[4], false,
        bpIgnited[5], true,
        bpIgnited[6], false);
    builder.Finish(message);
}