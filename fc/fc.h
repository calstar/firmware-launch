#include "pins.h"
#include "lib/MPL3115A2/MPL3115A2.h"

#define RS422_BAUDRATE (115200)
#define MSG_SEND_INTERVAL_US (100000u) // 100*1000us = 100ms
#define NUM_BP (7)
#define BUF_SIZE (1024)

using namespace flatbuffers;
using namespace Calstar;


void *readFlatbuffer(char c);
void buildCurrentMessage();

Timer msgTimer;
FlatBufferBuilder builder(BUF_SIZE);
bool bpIgnited[NUM_BP];
int main() {
    msgTimer.start();

    Serial rs422(RS422_TX, RS422_RX);
    rs422.baud(RS422_BAUDRATE);
    rs422.set_blocking(false);
    
    buildCurrentMessage();

    us_timestamp_t last_msg_send_us = msgTimer.read_high_resolution_us();
    while (true) {
        // Send a message every MSG_SEND_INTERVAL_US microseconds
        if (msgTimer.read_high_resolution_us() >= last_msg_send_us + MSG_SEND_INTERVAL_US) {
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

// #define LIST_LENGTH 50
// #define GROUND_SAMPLES 10000
// #define MIN_ALTITUDE 200
// #define MAIN_CHUTE_THRESHOLD 600
// #define LAUNCH_THRESHOLD 10

// // TODO: Turn off BP after certain time?

// typedef struct node {
//     struct node* previous;
//     double value;
// } node;

// typedef enum {
//     PRE_LAUNCH = 0,
//     ON_GROUND = 1,
//     LAUNCHED = 2,
//     PASSED_MIN_ALT = 3,
//     PASSED_APOGEE = 4,
//     PASSED_MAIN_CHUTE = 5
// } launch_state;

// node* root = 0;
// launch_state state = PRE_LAUNCH;

// I2C i2c_sensors(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
// MPL3115A2 altimeter(&i2c_sensors, &debug_talk);
// Altitude altitude;
// void read_sensors() {
//     altimeter.read_altitude(&altitude);
//     root = root->previous;
//     root->value = altitude.altitude();
// }

// Timer timer;
// float t_prev;
// float t_elapsed;
// void update_timer() {
//     float t_curr = timer.time();
//     t_elapsed = t_curr - t_prev;
//     t_prev = t_curr;
// }

// void send_telemetry() {
//     // TODO
// }

// DigitalOut led_r(STATE_LED_RED);
// DigitalOut led_g(STATE_LED_GREEN);
// DigitalOut led_b(STATE_LED_BLUE);
// void update_led() {
//     led_r.write(state & 0x1);
//     led_g.write(state & 0x10);
//     led_b.write(state & 0x100);
// }

// long ground_accumulated = 0;
// int samples = 0;
// void handle_pre_launch() {
//     ground_accumulated += root->value;
//     samples++;
//     if (samples >= GROUND_SAMPLES) {
//         float ground_altitude = ground_accumulated / samples;
//         altimeter.setOffsetAltitude(ground_altitude);
//         state = ON_GROUND;
//     }
// }

// void handle_on_ground() {
//     if (root->value > LAUNCH_THRESHOLD) {
//         state = LAUNCHED;
//     }
// }

// void handle_launched() {
//     if (root->value > MIN_ALTITUDE) state = PASSED_MIN_ALT;
// }

// void handle_passed_min_alt() {
//     // (a[1] - a[0]) + (a[2] - a[1]) + ... + (a[n] - a[n - 1]) = a[n] - a[0]
//     // Might not be the best way since it relies on two measurements
//     long slope_sum = root->previous->value - root->value;
//     if (slope_sum < 0) {
//         // Write BP
//         state = PASSED_APOGEE;
//     }
// }

// void handle_passed_apogee() {
//     if (root->value <= MAIN_CHUTE_THRESHOLD && root->previous->value >= MAIN_CHUTE_THRESHOLD) {
//         // BP
//         state = PASSED_MAIN_CHUTE;
//     }
// }

// float descent_elapsed = 0;
// void handle_passed_main_chute() {
// }

// int cycle() {
//     read_sensors();
//     update_timer();

//     switch (state)  {
//         case PRE_LAUNCH:
//             handle_pre_launch();
//             break;
//         case ON_GROUND:
//             handle_on_ground();
//             break;
//         case LAUNCHED:
//             handle_launched();
//             break;
//         case PASSED_MIN_ALT:
//             handle_passed_min_alt();
//             break;
//         case PASSED_APOGEE:
//             handle_passed_apogee();
//             break;
//         case PASSED_MAIN_CHUTE:
//             handle_passed_main_chute();
//             break;
//     }

//     send_telemetry();

//     update_led();

//     wait(0.001);

//     return 1;
// }

// int run() {
//     // Initializes altitude linked list
//     node first = { .value = 0, .previous = root };
//     for (int i = 0; i < LIST_LENGTH - 1; i++) {
//         node n = { .value = 0, .previous = root };
//         root = &n;
//     }
//     first.previous = root;

//     altimeter.init();

//     timer.start();

//     while (cycle()) {}

//     timer.stop();
// }