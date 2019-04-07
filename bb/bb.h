#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "MPL3115A2.h"

#include "mbed.h"
#include "pins.h"

using namespace Calstar;

DigitalOut led_red(STATE_LED_RED);
DigitalOut led_green(STATE_LED_GREEN);
DigitalOut led_blue(STATE_LED_BLUE);

Serial debug_uart(DEBUG_TX, DEBUG_RX, DEBUG_UART_BAUDRATE);

I2C mpl_i2c(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
MPL3115A2 alt(&mpl_i2c, &debug_uart);

int main() {
  led_green = 1;
  led_red = 1;
  led_blue = 1;

  debug_uart.printf("---- CalSTAR BLACK BOX ----\r\n");
  alt.init();
  debug_uart.printf("altimiter whoami: 0x%X\r\n", alt.whoAmI());

  Altitude a;
  Temperature t;

  while(1){
    alt.readAltitude(&a);
    alt.readTemperature(&t);
    led_green = 0;
    wait_ms(250);
    led_green = 1;
    debug_uart.printf("Altitude: %sft, Temp: %sºF\r\n", a.print(), t.print());
    wait_ms(250);
  }
}
