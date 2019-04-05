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
#include <inttypes.h>
#include <string>
#include <unordered_map>
#include <vector>

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
#define ACK_CHECK_INTERVAL_MS (1000)
#define MAX_NUM_RETRIES (50)

#define BATUINT_TO_FLOAT(a) (((float)(a))/3308.4751259079328064817304607171f)
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
int32_t t_last_resend;

uint8_t frame_id;

// frame_id, <buffer size, buffer, number of retries>
std::unordered_map<uint8_t, std::pair<std::vector<uint8_t>, uint8_t>>
    acks_remaining;

FlatBufferBuilder builder(FLATBUF_BUF_SIZE);

/***************Function Declarations***********/
void start();
void loop();
bool sendUplinkMsg(const std::string &str, bool with_ack);
const DownlinkMsg *getDownlinkMsgChar(char c);
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

  t_last_resend = t.read_ms();
}

void loop() {
  if (t.read_ms() - t_last_resend > ACK_CHECK_INTERVAL_MS) {
    resend_msgs();
    t_last_resend = t.read_ms();
  }
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
          pc.putc('a');
          sendUplinkMsg(line, true);
          t_tx_led_on = t.read_ms();
          pc.printf("\r\nOut of send up link message true\r\n");
        } else {
          pc.printf("![SENDING ONCE ' %s ', bytes: %d]!\r\n", line.c_str(),
                    line.length());
          line += '\n';
          tx_led = 1;
          pc.putc('b');
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
    bool failed = true;
    for (int32_t i = 0; i < num_bytes_rxd - 1; ++i) {
      const DownlinkMsg *msg = getDownlinkMsgChar(rx_buf[i + 1]);
      if (msg != nullptr) {
        failed = false;
        pc.printf("![RSSI=%d, bytes: %d]!", radio.getRSSI(), num_bytes_rxd - 1);
        if (msg->Type() == DownlinkType_Ack) {
          if (acks_remaining.count(msg->FrameID()) == 1) {
            acks_remaining.erase(msg->FrameID());
          }
          pc.printf("\r\n");
        } else if (msg->Type() == DownlinkType_StateUpdate) {
          pc.printf("tstamp: %" PRIu64 ", bytes: %d, state: %d, fc.pwr: %d, "
                    "gps string: \"%s\", bat.v: %f",
                    msg->TimeStamp(), (int)msg->Bytes(), (int)msg->State(),
                    (int)msg->FCPowered(), msg->GPSString()->str().c_str(),
                    BATUINT_TO_FLOAT(msg->BattVoltage()));
          if (msg->FCMsg()) {
            const FCUpdateMsg *fc = msg->FCMsg();
            pc.printf(
                ", ((state: %d, accel: %f, %f %f, mag: %f, %f, %f, gyro: %f, %f, %f, alt: %f, pressure: %f, bps: %d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d))",
                (int) fc->State(), fc->AccelX(), fc->AccelY(), fc->AccelZ(),
                fc->MagX(), fc->MagY(), fc->MagZ(),
                fc->GyroX(), fc->GyroY(), fc->GyroZ(),
                fc->Altitude(), fc->Pressure(),
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
          pc.printf("\r\n");
        }
        if (msg->AckReqd()) {
          sendAck(msg->FrameID());
        }
      } else {
      }
    }
    if (failed) {
      // pc.printf("[!RSSI=%d, bytes: %d]! Failed Flatbuf Deserialize\r\n",
      //           radio.getRSSI(), num_bytes_rxd - 1);
    }
  }
}

bool sendUplinkMsg(const std::string &str, bool with_ack) {
  pc.printf("0");
  builder.Reset();

  uint8_t bps[7] = {0, 0, 0, 0, 0, 0, 0};
  /* NEED TO ACTUALLY PARSE STR */
  UplinkType type = UplinkType_FCOff;
  if (str[0] == 'n') {
    type = UplinkType_FCOn;
  } else if (str[0] == 'b') {
    type = UplinkType_BlackPowderPulse;
    pc.printf("%s\r\n", str.c_str());
    if (str.length() >= 8) {
      for (int32_t i = 1; i < 8; ++i) {
        if (str[i] == '1') {
          bps[i - 1] = 1;
        } else {
          bps[i - 1] = 0;
        }
      }
    }

  } else if (str[0] == 'f') {
    type = UplinkType_FCOff;
  } else {
    type = UplinkType_FCOff;
  }

  auto blackpowder_offset = builder.CreateVector(bps, sizeof(bps));

  Offset<UplinkMsg> msg =
      CreateUplinkMsg(builder, 1, type, blackpowder_offset, frame_id, with_ack);
  builder.Finish(msg);

  const uint8_t bytes = (uint8_t)builder.GetSize();
  builder.Reset();
  blackpowder_offset = builder.CreateVector(bps, sizeof(bps));
  msg = CreateUplinkMsg(builder, bytes, type, blackpowder_offset, frame_id,
                        with_ack);
  builder.Finish(msg);

  const uint8_t *buf = builder.GetBufferPointer();
  const int32_t size = builder.GetSize();

  if (with_ack) {
    acks_remaining.insert(
        {frame_id, {std::vector<uint8_t>(buf, buf + size), 0}});
    pc.printf("1");
  }
  radio.send(buf, size);
  pc.printf("2");

  ++frame_id;
}

uint8_t ret_buf[FLATBUF_BUF_SIZE];
const DownlinkMsg *getDownlinkMsgChar(char c) {
  static uint8_t buffer[FLATBUF_BUF_SIZE];
  static unsigned int len = 0;

  if (len == FLATBUF_BUF_SIZE) {
    // If at end of buffer, shift and add to end
    memmove(buffer, buffer + 1, FLATBUF_BUF_SIZE - 1);
    buffer[FLATBUF_BUF_SIZE - 1] = (uint8_t)c;
  } else {
    // Otherwise build up buffer
    buffer[len++] = (uint8_t)c;
  }

  // The verifier will say that buf has a valid message for any length from
  // actual_length-some number to full buffer length So basically, we trust the
  // verifier, but verify separately by having a #-bytes field in the message
  // itself So if the verifier says there's a valid message in the buffer, we
  // read that message, get the number of bytes that the message says it should
  // be, and actually process a message of THAT size.
  Verifier verifier(buffer, len);
  if (VerifyDownlinkMsgBuffer(verifier)) {
    const DownlinkMsg *msg = GetDownlinkMsg(buffer);
    // The message knows how big it should be
    const uint8_t expectedBytes = msg->Bytes();

    uint8_t actual_len = len;
    if (len < expectedBytes) {
      // The verifier will say we have a valid message even if we're a few bytes
      // short Just read more characters at this point by returning early
      return nullptr;
    } else if (len > expectedBytes) {
      // Now we want to verify that the "smaller buffer" with length equal to
      // the expected number of bytes is actually a message in its own right
      // (just a double check basically)
      Verifier smallerVerifier(buffer, expectedBytes);
      if (VerifyDownlinkMsgBuffer(smallerVerifier)) {
        // If it is a message, then make sure we use the correct (smaller)
        // length
        actual_len = expectedBytes;
      } else {
        // If it isn't valid, then this buffer just has some malformed
        // messages... continue and let's get them out of the buffer by reading
        // more
        return nullptr;
      }
    }

    // Now that we've read a valid message, copy it into the output buffer,
    // then remove it from the input buffer and move everything else down.
    // Then reduce current buffer length by the length of the processed message
    // Then clear the rest of the buffer so that we don't get false positives
    // with the verifiers
    memcpy(ret_buf, buffer, actual_len);
    memmove(buffer, buffer + actual_len, FLATBUF_BUF_SIZE - actual_len);
    len -= actual_len;
    // Clear the rest of the buffer
    memset(buffer + len, 0, FLATBUF_BUF_SIZE - len);

    return GetDownlinkMsg(ret_buf);
  }
  return nullptr;
}

void resend_msgs() {
  bool resent = false;
  for (auto &msg : acks_remaining) {
    resent = true;
    tx_led = 1;
    pc.putc('c');
    pc.printf("![RESENDING FRAME '%d']!\r\n", (int)msg.first);
    const std::vector<uint8_t> &vec = std::get<0>(msg.second);
    radio.send(vec.data(), vec.size());
    // pc.printf("Complete\r\n");
    std::get<1>(msg.second) = std::get<1>(msg.second) + 1;
    if (std::get<1>(msg.second) >= MAX_NUM_RETRIES) {
      acks_remaining.erase(msg.first);
      pc.printf("Rmd an entry\r\n");
    }
  }
  if (resent) {
    t_tx_led_on = t.read_ms();
  }
}

void sendAck(uint8_t frame_id) {
  builder.Reset();
  Offset<UplinkMsg> ack =
      CreateUplinkMsg(builder, 1, UplinkType_Ack, 0, frame_id, false);
  builder.Finish(ack);
  const uint8_t bytes = builder.GetSize();
  builder.Reset();
  ack = CreateUplinkMsg(builder, bytes, UplinkType_Ack, 0, frame_id, false);
  builder.Finish(ack);
  radio.send(builder.GetBufferPointer(), builder.GetSize());
}