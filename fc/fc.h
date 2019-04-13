#include "msg_downlink_generated.h"
#include "msg_fc_update_generated.h"
#include "msg_uplink_generated.h"

#include "mbed.h"
#include "pins.h"

#include "MPL3115A2.h"

#define DEBUG_UART_BAUDRATE (115200)
#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (100000u) // 100*1000us = 100ms
#define NUM_BP (7)
#define BUF_SIZE (256)

using namespace flatbuffers;
using namespace Calstar;

const UplinkMsg *getUplinkMsg(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
bool bpIgnited[NUM_BP] = {false, false, true, false, false, false, true};

DigitalOut led_red(STATE_LED_RED);
DigitalOut led_green(STATE_LED_GREEN);
DigitalOut led_blue(STATE_LED_BLUE);

UARTSerial rs422(RS422_TX, RS422_RX, RS422_BAUDRATE);
Serial debug_uart(DEBUG_TX, DEBUG_RX, DEBUG_UART_BAUDRATE);

I2C mpl_i2c(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
MPL3115A2 alt(&mpl_i2c, &debug_uart);
float last_alt = 0;

us_timestamp_t last_msg_send_us;

uint8_t rs422_read_buf[BUF_SIZE];

void loop() {
    // Send a message every MSG_SEND_INTERVAL_US microseconds
    us_timestamp_t current_time = msgTimer.read_high_resolution_us();
    if (current_time >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
        buildCurrentMessage();

        uint8_t *buf = builder.GetBufferPointer();
        int size = builder.GetSize();

        rs422.write(buf, size);

        // also just toggle the red LED at this point
        led_red = led_red ? 0 : 1;

        last_msg_send_us = current_time;
    }
    // Always read messages
    while (rs422.readable()) {
        ssize_t num_read = rs422.read(rs422_read_buf, BUF_SIZE);
        for (int i = 0; i < num_read; i++) {
            const UplinkMsg *msg = getUplinkMsg(rs422_read_buf[i]);
            if (msg) {
                // If turning on/off black powder, for now we just set our state
                // (in the future we will actually do black powder stuff)
                if (msg->Type() == UplinkType_BlackPowderPulse) {
                    for (uoffset_t bp = 0;
                         bp < NUM_BP && bp < msg->BP()->size(); bp++) {
                        bpIgnited[bp] |= msg->BP()->Get(bp);
                    }

                    debug_uart.printf("Received BlackPowderPulse -->\r\n");
                    debug_uart.printf("    Current ignited state: ");
                    for (int bp = 0; bp < NUM_BP; bp++) {
                        debug_uart.printf("%d", bpIgnited[bp]);
                        if (bp != NUM_BP - 1) {
                            debug_uart.printf(",");
                        }
                    }
                    debug_uart.printf("\r\n");
                }
                led_green = !bpIgnited[0];
                led_blue = !bpIgnited[1];
            }
        }
    }

    if (debug_uart.readable()) {
        char c = debug_uart.getc();

        if (c == 'p') {
            debug_uart.printf("Current ignited state: ");
            for (int i = 0; i < NUM_BP; i++) {
                debug_uart.printf("%d", bpIgnited[i]);
                if (i != NUM_BP - 1) {
                    debug_uart.printf(",");
                }
            }
            debug_uart.printf("\r\n");
        }
    }

    Altitude alt_result;
    alt.readAltitude(&alt_result);
    last_alt = alt_result.altitude(Altitude::FEET);
    debug_uart.printf("Read altitude: %f ft\r\n", last_alt);
}

void start() {
    led_red = 0;
    led_green = !bpIgnited[0];
    led_blue = !bpIgnited[1];
    msgTimer.start();

    // rs422.set_blocking(true);

    debug_uart.set_blocking(false);
    debug_uart.printf("---- CalSTAR Flight Computer ----\r\n");

    debug_uart.printf("Initializing altimeter@0x%X:\r\n", MPL3115A2_ADDRESS);
    alt.init();
    alt.setOversampleRate(0b000);
    alt.setModeStandby();
    alt.setModeAltimeter();
    alt.setModeActive();
    debug_uart.printf("altimiter whoami: 0x%X\r\n", alt.whoAmI());

    buildCurrentMessage();

    last_msg_send_us = msgTimer.read_high_resolution_us();
}

int main() {
    start();
    while (1) {
        loop();
    }
}

uint8_t uplinkMsgBuffer[BUF_SIZE];
const UplinkMsg *getUplinkMsg(char c) {
    static uint8_t buffer[BUF_SIZE];
    static unsigned int len = 0;

    if (len == BUF_SIZE) {
        // If at end of buffer, shift and add to end
        memmove(buffer, buffer + 1, BUF_SIZE - 1);
        buffer[BUF_SIZE - 1] = (uint8_t)c;
    } else {
        // Otherwise build up buffer
        buffer[len++] = (uint8_t)c;
    }

    // The verifier will say that buf has a valid message for any length from
    // actual_length-some number to full buffer length So basically, we trust
    // the verifier, but verify separately by having a #-bytes field in the
    // message itself So if the verifier says there's a valid message in the
    // buffer, we read that message, get the number of bytes that the message
    // says it should be, and actually process a message of THAT size.
    Verifier verifier(buffer, len);
    if (VerifyUplinkMsgBuffer(verifier)) {
        const UplinkMsg *msg = GetUplinkMsg(buffer);
        // The message knows how big it should be
        uint8_t expectedBytes = msg->Bytes();

        uint8_t actual_len = len;
        if (len < expectedBytes) {
            // The verifier will say we have a valid message even if we're a few
            // bytes short Just read more characters at this point by returning
            // early
            return NULL;
        } else if (len > expectedBytes) {
            // Now we want to verify that the "smaller buffer" with length equal
            // to the expected number of bytes is actually a message in its own
            // right (just a double check basically)
            Verifier smallerVerifier(buffer, expectedBytes);
            if (VerifyUplinkMsgBuffer(smallerVerifier)) {
                // If it is a message, then make sure we use the correct
                // (smaller) length
                actual_len = expectedBytes;
            } else {
                // If it isn't valid, then this buffer just has some malformed
                // messages... continue and let's get them out of the buffer by
                // reading more
                return NULL;
            }
        }

        // Now that we've read a valid message, copy it into the output buffer,
        // then remove it from the input buffer and move everything else down.
        // Then reduce current buffer length by the length of the processed
        // message Then clear the rest of the buffer so that we don't get false
        // positives with the verifiers
        memcpy(uplinkMsgBuffer, buffer, actual_len);
        memmove(buffer, buffer + actual_len, BUF_SIZE - actual_len);
        len -= actual_len;
        // Clear the rest of the buffer
        memset(buffer + len, 0, BUF_SIZE - len);

        return GetUplinkMsg(uplinkMsgBuffer);
    }
    return NULL;
}

void buildCurrentMessage() {
    builder.Reset();
    Offset<FCUpdateMsg> message =
        CreateFCUpdateMsg(builder,
                          1, // Can't be 0 or it will be ignored
                          FCState_Pad,
                          // 0.0f, 1.0f, 2.0f,
                          // 3.0f, 4.0f, 5.0f,
                          // 6.0f, 7.0f, 8.0f,
                          last_alt, // 10.0f,
                          false, bpIgnited[0], true, bpIgnited[1], false,
                          bpIgnited[2], true, bpIgnited[3], false, bpIgnited[4],
                          true, bpIgnited[5], false, bpIgnited[6]);
    builder.Finish(message);

    uint8_t bytes = (uint8_t)builder.GetSize();
    builder.Reset();
    message =
        CreateFCUpdateMsg(builder,
                          bytes, // Fill in actual number of bytes
                          FCState_Pad,
                          // 0.0f, 1.0f, 2.0f,
                          // 3.0f, 4.0f, 5.0f,
                          // 6.0f, 7.0f, 8.0f,
                          last_alt, // 10.0f,
                          false, bpIgnited[0], true, bpIgnited[1], false,
                          bpIgnited[2], true, bpIgnited[3], false, bpIgnited[4],
                          true, bpIgnited[5], false, bpIgnited[6]);
    builder.Finish(message);
}
