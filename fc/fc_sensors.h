#include "pins.h"
#include "mbed.h"
#include "MPL3115A2.h"

#define METERS(x) (x * 0.3048)
#define LIST_LENGTH 50
#define GROUND_SAMPLES 10000
#define MIN_ALTITUDE METERS(200)
#define MAIN_CHUTE_THRESHOLD METERS(600)
#define LAUNCH_THRESHOLD METERS(10)

// TODO: Merge this into the main fc.h file
// For now just using it to test SIL

// TODO: Turn off BP after certain time?

struct Node {
    Node* previous;
    Node* next;
    double value;

    Node(Node* p, double v) : previous(p), value(v) {}

    static Node* createList(int length) {
        Node* first = new Node(nullptr, 0);
        Node* last = first; 
        for (int i = 0; i < length; i++) {
            last = new Node(last, 0);
            last->previous->next = last;
        }
        first->previous = last;
        last->next = first;
        return first;
    }
};

enum LaunchState {
    PRE_LAUNCH = 0b011,
    ON_GROUND = 0b101,
    LAUNCHED = 0b110,
    PASSED_MIN_ALT = 0b001,
    PASSED_APOGEE = 0b010,
    PASSED_MAIN_CHUTE = 0b100
};

Node* root = nullptr;
Node* last = nullptr;
LaunchState state = PRE_LAUNCH;

Serial debug_talk(DEBUG_RX, DEBUG_TX);
I2C i2c_sensors(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
MPL3115A2 altimeter(&i2c_sensors, &debug_talk);
Altitude altitude;

void read_sensors() {
    altimeter.readAltitude(&altitude);
    root = root->previous;
    root->value = altitude.altitude();
}

Timer timer;
us_timestamp_t t_prev;
us_timestamp_t t_elapsed;
void update_timer() {
    us_timestamp_t t_curr = timer.read_high_resolution_us();
    t_elapsed = t_curr - t_prev;
    t_prev = t_curr;
}

void send_telemetry() {
    // TODO
}

DigitalOut led_r(STATE_LED_RED);
DigitalOut led_g(STATE_LED_GREEN);
DigitalOut led_b(STATE_LED_BLUE);
void update_led() {
    led_r.write(state & 0x1);
    led_g.write(state & 0x10);
    led_b.write(state & 0x100);
}

long ground_accumulated = 0;
int samples = 0;
void handle_pre_launch() {
    ground_accumulated += root->value;
    samples++;
    if (samples >= GROUND_SAMPLES) {
        printf("State reached: On ground\n");
        float ground_altitude = ground_accumulated / samples;
        altimeter.setOffsetAltitude(-ground_altitude);
        state = ON_GROUND;
    }
}

void handle_on_ground() {
    if (root->value > LAUNCH_THRESHOLD) {
        printf("State reached: Launched\n");
        state = LAUNCHED;
    }
}

void handle_launched() {
    if (root->value > MIN_ALTITUDE) {
        printf("State reached: Passed min altitude\n");
        state = PASSED_MIN_ALT;
    }
}

void handle_passed_min_alt() {
    // (a[1] - a[0]) + (a[2] - a[1]) + ... + (a[n] - a[n - 1]) = a[n] - a[0]
    // Might not be the best way since it relies on two measurements
    float slope_sum = root->next->value - root->value;
    if (slope_sum < 0) {
        printf("State reached: Passed apogee\n");
        // Write drogue
        state = PASSED_APOGEE;
    }
}

void handle_passed_apogee() {
    if (root->value <= MAIN_CHUTE_THRESHOLD && root->previous->value >= MAIN_CHUTE_THRESHOLD) {
        printf("State reached: Deployed main chute\n");
        // Write main
        state = PASSED_MAIN_CHUTE;
    }
}

float descent_elapsed = 0;
void handle_passed_main_chute() {
}

void loop() {
    read_sensors();
    update_timer();

    switch (state)  {
        case PRE_LAUNCH:
            handle_pre_launch();
            break;
        case ON_GROUND:
            handle_on_ground();
            break;
        case LAUNCHED:
            handle_launched();
            break;
        case PASSED_MIN_ALT:
            handle_passed_min_alt();
            break;
        case PASSED_APOGEE:
            handle_passed_apogee();
            break;
        case PASSED_MAIN_CHUTE:
            handle_passed_main_chute();
            break;
    }

    send_telemetry();

    update_led();
}

void start() {
    // Initializes altitude linked list
    root = Node::createList(LIST_LENGTH);

    altimeter.init();

    timer.start();
}