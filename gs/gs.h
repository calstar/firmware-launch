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
#include <vector>

using namespace flatbuffers;
using namespace Calstar;

/****************Defines**********************/
#define ENCRYPT_KEY ("CALSTARENCRYPTKE")

#define COMMAND_YES_RETRY ("![YES_RETRY]!")
#define COMMAND_NO_RETRY ("![NO_RETRY]!")

#define LED_ON_TIME_MS (50)

#define RX_BUF_LEN (128)

#define FLATBUF_BUF_SIZE (256)

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

std::vector<std::tuple<uint32_t, uint8_t *, uint8_t>> acks_remaining;

/***************Function Declarations***********/
void start();
void loop();
bool sendUplinkMsg(const std::string &str, bool with_ack);
const DownlinkMsg *getDownlinkMsg(uint8_t *data, int32_t data_len);

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
          pc.printf("![SENDING ONCE (RETRY TODO) '%s', bytes: %d]!\r\n",
                    line.c_str(), line.length());
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
    pc.printf("![RSSI=%d, bytes: %d]! %s\r\n", radio.getRSSI(),
              num_bytes_rxd - 1, rx_buf + 1);
    t_rx_led_on = t.read_ms();
  }
}

bool sendUplinkMsg(const std::string &str, bool with_ack) {
  radio.send(line.c_str(), line.length());
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