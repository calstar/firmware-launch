/*
 * CalSTAR Avionics Ground Station
 *
 * file: gs.h
 *
 * Provides communication between laptop and rocket.
 * Communicates with laptop via UART over USB (with UART with an FTDI cable as
 * backup) Communicates with rocket via radio, 433 MHz Has 2 configurable LEDs
 * and 4 digital inputs on terminal blocks
 *
 * LEDS:
 *  - Rx (PB12)
 *  - Tx (PB13)
 *
 * Digital Inputs:
 *  - IO 1 (PB5)
 *  - IO 2 (PB6)
 *  - IO 3 (PB8) // NOTE: only usable on v3.1+
 *  - IO 4 (PB7)
 */

/* Includes */

#include "pins.h"

#include "mbed.h"
#include "USBSerial.h"
#include "RFM69/RFM69.hpp"
#include "json_messages.h"
#include "msg_downlink_generated.h"
#include "msg_fc_update_generated.h"
#include "msg_uplink_generated.h"

using namespace flatbuffers;
using namespace Calstar;

#include <inttypes.h>
#include <string>
#include <unordered_map>
#include <vector>

struct TelecmdMsg {
    std::vector<uint8_t> buf;
    uint8_t frame_id;
    uint8_t num_retries;
};

/* Defines */

// Must be exactly 16 bytes (excluding null-terminator).
//     Therefore is missing the 'Y' on purpose. 
#define ENCRYPT_KEY ("CALSTARENCRYPTKE")

#define LED_ON_TIME_MS (50)

#define RX_BUF_LEN (256)

#define JSON_LOG_UPDATE_INTERVAL_MS (200)

#define ACK_RESEND_INTERVAL_MS (200)
#define MAX_NUM_RETRIES (50)

#define COMMAND_YES_RETRY ("![YES_RETRY]!")
#define COMMAND_NO_RETRY ("![NO_RETRY]!")

#define FLATBUF_BUF_SIZE (256)

#define BATUINT_TO_FLOAT(a) (((float)(a)) / 3308.4751259079328064817304607171f)

/* Function Declarations */
void init_leds();
void init_radio();
void init_inputs();

void update_leds();
void update_inputs();

void periodic_json_log_update();

void resend_missing_acks();

bool execute_command(const std::string &cmd);
bool parse_to_telecmd_msg(const std::string &line, const uint8_t frame_id, const bool ack_req, std::vector<uint8_t> *const out);

void radio_transmit(const uint8_t *const buf, const int32_t size);

const DownlinkMsg *stream_parse_to_downlink(const uint8_t *const buf, const size_t buf_size);
void push_to_stream(const uint8_t b, uint8_t *const buf, size_t *const buf_size);
const DownlinkMsg *parse_to_downlink(uint8_t *const buf, size_t *const buf_size);

void handle_incoming_msg(const DownlinkMsg *const msg);
void handle_acks(const uint8_t frame_id);
void send_ack(const uint8_t frame_id);

void json_log_to_serial(const DownlinkMsg *const msg);

void sendJsonInt(uint64_t timestamp, const std::string &id, int value);
void sendJsonFloat(uint64_t timestamp, const std::string &id, float value);
void sendJsonString(uint64_t timestamp, const std::string &id, const std::string &value);
void sendJsonLog(const std::string &value);

/* Global Variables */
Timer t;

DigitalOut rx_led(LED_RX);
DigitalOut tx_led(LED_TX);
DigitalOut tx_lock_led(IO2);
int32_t t_tx_led_on;
int32_t t_rx_led_on;

DigitalIn tx_lock_button(IO1);

uint32_t total_bytes_sent;

uint8_t tx_frame_id;

int32_t t_last_ack_resend;
int32_t t_last_periodic_log;

uint8_t rx_buf[RX_BUF_LEN];
RFM69 radio(SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_SSEL, RADIO_RST, true);
bool radio_retry;

USBSerial serial;

std::string line;

std::unordered_map<uint8_t, TelecmdMsg> acks_remaining;

FlatBufferBuilder builder(FLATBUF_BUF_SIZE);
uint8_t out_buf[FLATBUF_BUF_SIZE];

/* Function Definitions */
void start() {
    t.start();

    init_leds();
    init_radio();
    init_inputs();

    tx_frame_id = 0;
    total_bytes_sent = 0;
    t_last_ack_resend = t.read_ms();
    t_last_periodic_log = t.read_ms();
    radio_retry = true;
    line = "";
    sendJsonLog("Start: Initialization complete.");
}

void loop() {
    update_leds();
    update_inputs();

    if (t.read_ms() - t_last_periodic_log > JSON_LOG_UPDATE_INTERVAL_MS) {
        periodic_json_log_update();
        t_last_periodic_log = t.read_ms();
    }

    if (t.read_ms() - t_last_ack_resend > ACK_RESEND_INTERVAL_MS) {
        resend_missing_acks();
        t_last_ack_resend = t.read_ms();
    }

    if (serial.readable()) {
        const char c = serial.getc();
        if (c == '\n') {
            if (!execute_command(line)) {
                sendJsonLog("Failed to execute command.");
            }
            line = "";
        } else {
            line += c;
        }
    }

    const int32_t num_bytes_rxd = radio.receive((char *)rx_buf, sizeof(rx_buf));
    if (num_bytes_rxd > 1) {
        rx_buf[num_bytes_rxd] = '\0';
        
        rx_led = 1;
        t_rx_led_on = t.read_ms();

        const DownlinkMsg *msg = stream_parse_to_downlink(rx_buf + 1, sizeof(rx_buf) - 1);
        if (msg != nullptr) {
            handle_incoming_msg(msg);
        }
    }
}

void init_leds() {
    rx_led = 0;
    tx_led = 0;
    tx_lock_led = 0;

    t_tx_led_on = t.read_ms();
    t_rx_led_on = t.read_ms();

    sendJsonLog("Leds: Turned off.");
}

void init_radio() {
    radio.reset();
    radio.init();
    radio.setAESEncryption(ENCRYPT_KEY, strlen(ENCRYPT_KEY));
    radio.setHighPowerSettings(true);
    radio.setPowerDBm(20);
    sendJsonLog("Radio: Initialized.");
}

void init_inputs() {
    // Set to active-low using internal pull-up
    tx_lock_button.mode(PullUp);
    sendJsonLog("Tx Lock Button: Pulled up");
}

void update_leds() {
    if (tx_led.read() == 1 && t.read_ms() - t_tx_led_on > LED_ON_TIME_MS) {
        tx_led = 0;
    }
    if (rx_led.read() == 1 && t.read_ms() - t_rx_led_on > LED_ON_TIME_MS) {
        rx_led = 0;
    }
}

void update_inputs() {
    if (tx_lock_button) {
        tx_lock_led = 1;
    } else {
        tx_lock_led = 0;
    }
}

void periodic_json_log_update() {
    sendJsonInt(-1, COMMS_SENT, total_bytes_sent);
}

void resend_missing_acks() {
    for (auto &kv : acks_remaining) {
        TelecmdMsg &msg = kv.second;
        radio_transmit(msg.buf.data(), msg.buf.size());
        ++msg.num_retries;
        if (msg.num_retries >= MAX_NUM_RETRIES) {
            acks_remaining.erase(msg.frame_id);
            sendJsonLog("Failed to send frame " + std::to_string(msg.frame_id) 
                + ": Exceeded max number of retries.");
        }
    }
}

bool execute_command(const std::string &cmd) {
    if (line == COMMAND_YES_RETRY) {
        radio_retry = true;
        sendJsonLog("Radio: Set to transmit with retry.");
    } else if (line == COMMAND_NO_RETRY) {
        radio_retry = false;
        sendJsonLog("Radio: Set to transmit without retry.");
    } else  {
        if (!tx_lock_button) {
            line += '\n';

            TelecmdMsg msg;
            msg.frame_id = tx_frame_id;
            msg.num_retries = 0;
            if (!parse_to_telecmd_msg(cmd, msg.frame_id, radio_retry, &msg.buf)) {
                sendJsonLog("Failed to send frame: unable to parse input.");
                return false;
            }
            ++tx_frame_id;

            if (radio_retry) {
                acks_remaining.insert({msg.frame_id, msg});
            }
            radio_transmit(msg.buf.data(), msg.buf.size());
        } else {
            sendJsonLog("Failed to send frame: Transmit locked.");
        }
    }
    return true;
}

bool parse_to_telecmd_msg(const std::string &line, const uint8_t frame_id, const bool ack_req, std::vector<uint8_t> *const out) {
    UplinkType type = UplinkType_FCOff;
    uint8_t bps[7] = {0, 0, 0, 0, 0, 0, 0};

    if (line[0] == 'n') {
        type = UplinkType_FCOn;
    } else if (line[0] == 'o') {
        type = UplinkType_FCOff;
    } else if (line[0] == 'b') {
        type = UplinkType_BlackPowderPulse;
        if (line.length() >= 8) {
            for (int32_t i = 1; i < 8; ++i) {
                if (line[i] == '1') {
                    bps[i - 1] = 1;
                } else if (line[i] == '0') {
                    bps[i - 1] = 0;
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    } else {
        return false;
    }

    builder.Reset();
    auto bps_offset = builder.CreateVector(bps, sizeof(bps));
    Offset<UplinkMsg> msg = CreateUplinkMsg(builder, 1, type, bps_offset, frame_id, ack_req);
    builder.Finish(msg);

    const uint8_t bytes = (uint8_t)builder.GetSize();
    builder.Reset();
    bps_offset = builder.CreateVector(bps, sizeof(bps));
    msg = CreateUplinkMsg(builder, bytes, type, bps_offset, frame_id, ack_req);
    builder.Finish(msg);

    const uint8_t *const buf = builder.GetBufferPointer();
    const int32_t size = builder.GetSize();

    *out = std::vector<uint8_t>(buf, buf + size);

    return true;
}

void radio_transmit(const uint8_t *const buf, const int32_t size) {
    tx_led = 1;
    radio.send(buf, size);
    t_tx_led_on = t.read_ms();
    total_bytes_sent += size;
}

const DownlinkMsg *stream_parse_to_downlink(const uint8_t *const buf, const size_t buf_size) {
    static uint8_t stream[FLATBUF_BUF_SIZE];
    static size_t stream_len = 0;

    const DownlinkMsg *msg = nullptr;
    for (size_t i = 0; i < buf_size; ++i) {
        push_to_stream(buf[i], stream, &stream_len);
        
        msg = parse_to_downlink(stream, &stream_len);
        if (msg != nullptr) {
            return msg;
        }
    }
    return nullptr;
}

void push_to_stream(const uint8_t b, uint8_t *const buf, size_t *const buf_size) {
    if (*buf_size == FLATBUF_BUF_SIZE) {
        memmove(buf, buf + 1, FLATBUF_BUF_SIZE - 1);
        buf[FLATBUF_BUF_SIZE - 1] = b;
    } else {
        buf[*buf_size] = b;
        ++(*buf_size);
    }
}

const DownlinkMsg *parse_to_downlink(uint8_t *const buf, size_t *const buf_size) {
    Verifier verifier(buf, *buf_size);
    if (VerifyDownlinkMsgBuffer(verifier)) {
        const DownlinkMsg *const msg = GetDownlinkMsg(buf);
        const uint8_t expected_num_bytes = msg->Bytes();

        uint8_t actual_size = *buf_size;
        if (*buf_size < expected_num_bytes) {
            return nullptr;
        } else if (*buf_size > expected_num_bytes) {
            Verifier smaller_verifier(buf, expected_num_bytes);
            if (VerifyDownlinkMsgBuffer(smaller_verifier)) {
                actual_size = expected_num_bytes;
            } else {
                return nullptr;
            }
        }

        memcpy(out_buf, buf, actual_size);
        memmove(buf, buf + actual_size, FLATBUF_BUF_SIZE - actual_size);
        *buf_size -= actual_size;
        memset(buf + *buf_size, 0, FLATBUF_BUF_SIZE - *buf_size);

        return GetDownlinkMsg(out_buf);
    }
    return nullptr;
}

void handle_incoming_msg(const DownlinkMsg *const msg) {
    if (msg->AckReqd()) {
        send_ack(msg->FrameID());
    }

    switch (msg->Type()) {
        case DownlinkType_Ack:
            handle_acks(msg->FrameID());
            break;
        case DownlinkType_StateUpdate:
            json_log_to_serial(msg);
            break;
        default:
            break;
    }
}

void send_ack(const uint8_t frame_id) {
    builder.Reset();
    Offset<UplinkMsg> ack = CreateUplinkMsg(builder, 1, UplinkType_Ack, 0, frame_id, false);
    builder.Finish(ack);
    const uint8_t bytes = builder.GetSize();
    builder.Reset();
    ack = CreateUplinkMsg(builder, bytes, UplinkType_Ack, 0, frame_id, false);
    builder.Finish(ack);
    radio_transmit(builder.GetBufferPointer(), builder.GetSize());
}

void handle_acks(const uint8_t frame_id) {
    if (acks_remaining.count(frame_id) >= 1) {
        acks_remaining.erase(frame_id);
    }
}

void json_log_to_serial(const DownlinkMsg *const msg) {
    const uint64_t tstamp = msg->TimeStamp();
    sendJsonInt(   tstamp,  COMMS_RECD,  msg->Bytes());
    sendJsonInt(   tstamp,  TPC_STATE,   (int)msg->State());
    sendJsonInt(   tstamp,  FC_PWRD,     (int)msg->FCPowered());
    sendJsonString(tstamp,  TPC_GPS,     msg->GPSString()->c_str());
    sendJsonFloat( tstamp,  TPC_BATV,    BATUINT_TO_FLOAT(msg->BattVoltage()));

    if (msg->FCMsg()) {
        const FCUpdateMsg *const fc = msg->FCMsg();
        sendJsonInt(  tstamp,  FC_STATE, (int)fc->State());
        sendJsonFloat(tstamp,  FC_ALT,   fc->Altitude());
    }
}

void sendJsonInt(uint64_t timestamp, const std::string &id, int value) {
    serial.printf("{ \"timestamp\": %" PRIu64 ", \"id\": \"%s\", \"value\": %d}\r\n", timestamp, id.c_str(), value);
}

void sendJsonFloat(uint64_t timestamp, const std::string &id, float value) {
    serial.printf("{ \"timestamp\": %" PRIu64 ", \"id\": \"%s\", \"value\": %f}\r\n", timestamp, id.c_str(), value);
}

void sendJsonString(uint64_t timestamp, const std::string &id, const std::string &value) {
    serial.printf("{ \"timestamp\": %" PRIu64 ", \"id\": \"%s\", \"value\": \"%s\"}\r\n", timestamp, id.c_str(), value.c_str());
}

void sendJsonLog(const std::string &value) {
    serial.printf("{\"timestamp\": -1, \"id\": \"%s\", \"value\": \"%s\"}\r\n", GS_LOG, value.c_str());
}

int main() {
    start();
    while (true) {
        loop();
    }
    return 0;
}