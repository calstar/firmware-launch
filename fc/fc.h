#include "pins.h"

#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (100000u) // 100*1000us = 100ms
#define NUM_BP (7)
#define BUF_SIZE (1024)

using namespace flatbuffers;
using namespace Calstar;


void *readFlatbuffer(char c);
void buildCurrentMessage();

Timer timer;
FlatBufferBuilder builder(BUF_SIZE);
bool bpIgnited[NUM_BP];
int main() {
    timer.start();

    Serial rs422(RS422_TX, RS422_RX);
    rs422.baud(RS422_BAUDRATE);
    rs422.set_blocking(false);
    
    buildCurrentMessage();

    us_timestamp_t last_msg_send_us = timer.read_high_resolution_us();
    while (true) {
        // Send a message every MSG_SEND_INTERVAL_US microseconds
        if (timer.read_high_resolution_us() >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
            uint8_t* buf = builder.GetBufferPointer();
            int size = builder.GetSize();
            void* size_ptr = (void*)(&size);
            // Write the size of the message
            rs422.write((uint8_t*)size_ptr, sizeof(size), NULL);
            // then write the message
            rs422.write(buf, size, NULL);
        }
        // Always read messages
        if (rs422.readable()) {
            // Read into message type
            void *data = readFlatbuffer(rs422.getc());
            const UplinkMsg *msg = GetUplinkMsg(data);
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
}

void *readFlatbuffer(char c) {
    static char buffer[BUF_SIZE];
    static unsigned int pos = 0;
    static unsigned int messageSize = 0;

    buffer[pos++] = c;
    if (pos == sizeof(int)) {
        // Read the size of the incoming message
        void *buf = buffer;
        messageSize = *((int*)buf);
    } else if (pos == sizeof(int) + messageSize) {
        // Reached end of message
        void *result = buffer + sizeof(int);
        pos = 0;
        messageSize = 0;
        return result;
    }

    return NULL;
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