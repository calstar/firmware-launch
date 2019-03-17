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

/***************Includes**********************/
#include "msg_downlink_generated.h"
#include "msg_fc_update_generated.h"
#include "msg_uplink_generated.h"

#include "mbed.h"
#include "pins.h"

#include "RFM69/RFM69.hpp"
#include "USBSerial.h"
#include <string>
#include <unordered_map>

using namespace flatbuffers;
using namespace Calstar;

/****************Defines**********************/
#define ENCRYPT_KEY ("CALSTARENCRYPTKE")

#define COMMAND_YES_RETRY ("![YES_RETRY]!")
#define COMMAND_NO_RETRY ("![NO_RETRY]!")

#define LED_ON_TIME_MS (50)

#define RX_BUF_LEN (256)
#define FLATBUF_BUF_SIZE (256)

// resend messages for which have not received acks at 50ms intervals
#define ACK_CHECK_INTERVAL_SEC (0.05f)
#define MAX_NUM_RETRIES (50)

/****************Global Variables***************/
DigitalOut rx_led(LED_RX);
DigitalOut tx_led(LED_TX);
DigitalIn io1(IO1);
DigitalIn io2(IO2);
DigitalIn io3(IO3);
DigitalIn io4(IO4);
Timer t;

USBSerial pc;

uint8_t rx_buf[RX_BUF_LEN];
RFM69 radio(SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_SSEL, RADIO_RST, true);

std::string line = "";
bool retry = true;

int32_t t_tx_led_on;
int32_t t_rx_led_on;

uint8_t frame_id;

// USB does not work with LowPowerTicker
Ticker ack_checker;

// frame_id, <buffer size, buffer, number of retries>
std::unordered_map<uint8_t, std::tuple<int32_t, uint8_t *, uint8_t>>
    acks_remaining;

FlatBufferBuilder builder(FLATBUF_BUF_SIZE);

/***************Function Declarations***********/
void start();
void loop();
bool sendUplinkMsg(const std::string &str, bool with_ack);
const DownlinkMsg *getDownlinkMsg(uint8_t *data, int32_t data_len);
void resend_msgs();
void sendAck(uint8_t frame_id);

int main() {
  start();
  while (true) {
    loop();
  }
  return 0;
}

void start() {
  rx_led = 0;
  tx_led = 0;

  radio.reset();
  pc.printf("![Radio reset complete.]!\r\n");

  radio.init();
  radio.setAESEncryption(ENCRYPT_KEY, strlen(ENCRYPT_KEY));

  radio.setHighPowerSettings(true);
  radio.setPowerDBm(20);

  t.start();
  t_tx_led_on = t.read_ms();
  t_rx_led_on = t.read_ms();

  frame_id = 0;

  ack_checker.attach(&resend_msgs, ACK_CHECK_INTERVAL_SEC);
}

void loop() {
  if (tx_led.read() == 1 && t.read_ms() - t_tx_led_on > LED_ON_TIME_MS) {
    tx_led = 0;
  }
  if (rx_led.read() == 1 && t.read_ms() - t_rx_led_on > LED_ON_TIME_MS) {
    rx_led = 0;
  }

  if (pc.readable()) {
    const char in = pc.getc();
    if (in == '\n') {
      if (line == COMMAND_YES_RETRY) {
        retry = true;
        pc.printf("%s\r\n", line.c_str());
      } else if (line == COMMAND_NO_RETRY) {
        retry = false;
        pc.printf("%s\r\n", line.c_str());
      } else {
        if (retry) {
          pc.printf("![SENDING WITH RETRY '%s', bytes: %d]!\r\n", line.c_str(),
                    line.length());
          line += '\n';
          tx_led = 1;
          sendUplinkMsg(line, true);
          t_tx_led_on = t.read_ms();
        } else {
          pc.printf("![SENDING ONCE ' %s ', bytes: %d]!\r\n", line.c_str(),
                    line.length());
          line += '\n';
          tx_led = 1;
          sendUplinkMsg(line, false);
          t_tx_led_on = t.read_ms();
        }
      }
      line = "";
    } else {
      line += in;
    }
  }

  const int32_t num_bytes_rxd = radio.receive((char *)rx_buf, sizeof(rx_buf));
  if (num_bytes_rxd > 1) {
    rx_buf[num_bytes_rxd] = '\0';
    rx_led = 1;
    t_rx_led_on = t.read_ms();
    const DownlinkMsg *msg = getDownlinkMsg(rx_buf + 1, num_bytes_rxd - 1);
    if (msg != nullptr) {
      pc.printf("![RSSI=%d, bytes: %d]!", radio.getRSSI(), num_bytes_rxd - 1);
      if (msg->Type() == DownlinkType_Ack) {
        if (acks_remaining.count(msg->FrameID()) == 1) {
          acks_remaining.erase(msg->FrameID());
        }
        pc.printf("\r\n");
      } else if (msg->Type() == DownlinkType_StateUpdate) {
        pc.printf("tstamp: %d, bytes: %d, state: %d, fc.pwr: %d, gps string: "
                  "%s, bat.v: %f\r\n",
                  msg->TimeStamp(), msg->Bytes(), msg->State(),
                  msg->FCPowered(), msg->GPSString(), msg->BattVoltage());
      }
      if (msg->AckReqd()) {
        sendAck(msg->FrameID());
      }
    }
  }
}

bool sendUplinkMsg(const std::string &str, bool with_ack) {
  builder.Reset();

  /* NEED TO ACTUALLY PARSE STR */
  UplinkType type = UplinkType_FCOff;
  if (str[0] == 'o') {
    type = UplinkType_FCOn;
  } else if (str[0] == 'b') {
    type = UplinkType_BlackPowderPulse;
  }

  pc.printf("1");
  uint8_t bps[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bps[4] = 1;
  auto blackpowder_offset = builder.CreateVector(bps, sizeof(bps));
  pc.printf("2");

  Offset<UplinkMsg> msg = CreateUplinkMsg(
      builder, 1, UplinkType_FCOff, blackpowder_offset, frame_id, with_ack);
  pc.printf("3");
  builder.Finish(msg);
  pc.printf("4");

  uint8_t bytes = (uint8_t)builder.GetSize();
  pc.printf("5");
  builder.Reset();
  pc.printf("6");
  blackpowder_offset = builder.CreateVector(bps, sizeof(bps));
  msg = CreateUplinkMsg(builder, bytes, UplinkType_FCOff, blackpowder_offset,
                        frame_id, with_ack);
  pc.printf("7");
  builder.Finish(msg);
  pc.printf("8");

  uint8_t *buf = builder.GetBufferPointer();
  pc.printf("9");
  int32_t size = builder.GetSize();
  pc.printf("a");

  if (with_ack) {
    acks_remaining.insert({frame_id, {size, buf, 0}});
    pc.printf("11");
  }
  radio.send(buf, size);
  pc.printf("12");

  ++frame_id;
}

const DownlinkMsg *getDownlinkMsg(uint8_t *data, int32_t data_len) {
  static uint8_t buf[FLATBUF_BUF_SIZE];
  static uint8_t return_buf[FLATBUF_BUF_SIZE];
  static uint32_t len = 0;

  uint32_t bytes_left = FLATBUF_BUF_SIZE - len;

  if (data_len > bytes_left) {
    uint32_t num_move = data_len - bytes_left;
    memmove(buf, buf + num_move, len - num_move);
    memcpy(buf + len - num_move, data, data_len);
    len = FLATBUF_BUF_SIZE;
  } else {
    memcpy(buf + len, data, data_len);
    len += data_len;
  }

  Verifier verifier(buf, len);
  if (VerifyDownlinkMsgBuffer(verifier)) {
    memcpy(return_buf, buf, len);
    const DownlinkMsg *output = GetDownlinkMsg(return_buf);

    uint8_t expected_num_bytes = output->Bytes();
    uint8_t actual_len = len;
    if (len < expected_num_bytes) {
      return NULL;
    } else if (len > expected_num_bytes) {
      Verifier smaller_verifier(buf, expected_num_bytes);
      if (VerifyDownlinkMsgBuffer(smaller_verifier)) {
        actual_len = expected_num_bytes;
        memset(return_buf + actual_len, 0, len - actual_len);
      } else {
        return nullptr;
      }
    }

    memmove(buf, buf + actual_len, FLATBUF_BUF_SIZE - actual_len);
    len -= actual_len;
    memset(buf + len, 0, FLATBUF_BUF_SIZE - len);

    return output;
  }
  return nullptr;
}

void resend_msgs() {
  for (auto &msg : acks_remaining) {
    radio.send(std::get<1>(msg.second), std::get<0>(msg.second));
    ++std::get<2>(msg.second);
    if (std::get<2>(msg.second) >= MAX_NUM_RETRIES) {
      acks_remaining.erase(msg.first);
    }
  }
}

void sendAck(uint8_t frame_id) {
  builder.Reset();
  Offset<UplinkMsg> ack =
      CreateUplinkMsg(builder, 1, UplinkType_Ack, 0, frame_id, false);
  builder.Finish(ack);
  uint8_t bytes = builder.GetSize();
  builder.Reset();
  ack = CreateUplinkMsg(builder, bytes, UplinkType_Ack, 0, frame_id, false);
  builder.Finish(ack);
  radio.send(builder.GetBufferPointer(), builder.GetSize());
}