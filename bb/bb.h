#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "MPL3115A2.h"

#include "mbed.h"
#include "pins.h"

#include <stdint.h>

using namespace Calstar;

DigitalOut led_red(STATE_LED_RED);
DigitalOut led_green(STATE_LED_GREEN);
DigitalOut led_blue(STATE_LED_BLUE);

Serial debug_uart(DEBUG_TX, DEBUG_RX, DEBUG_UART_BAUDRATE);

I2C mpl_i2c(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
MPL3115A2 alt(&mpl_i2c, &debug_uart);

SPI kx222(KX222_MOSI, KX222_MISO, KX222_SCLK, KX222_CS);
//DigitalOut kx222_cs(KX222_CS);

uint8_t regread(SPI *spi, uint8_t reg);
void regwrite(SPI *spi, uint8_t reg, uint8_t payload);

int main() {
  uint8_t altimiter = alt.whoAmI();
  uint8_t kxwhoami;
  led_green = 1;
  led_red = 1;
  led_blue = 1;

  debug_uart.printf("---- CalSTAR BLACK BOX ----\r\n");
  alt.init();
  debug_uart.printf("altimiter whoami: 0x%X\r\n", altimiter);
  
  kx222.format(8,1);
  kx222.frequency(100000);

  regwrite(&kx222, 0x18, 0x80);
  kxwhoami = regread(&kx222, 0x0F);
  debug_uart.printf("kx222 whoami: %d\r\n", kxwhoami);

  Altitude a;
  Temperature t;

  while(1){
    if(altimiter != 0xFF){
      alt.readAltitude(&a);
      alt.readTemperature(&t);
      debug_uart.printf("Altitude: %sft, Temp: %sºF\r\n", a.print(), t.print());
    }
    led_green = 0;
    wait_ms(250);
    led_green = 1;
    wait_ms(250);
  }
}

uint8_t regread(SPI *spi, uint8_t reg){
  uint8_t out;
  spi->write(reg | 0x80);
  out = spi->write(0x00);

  return out;
}

void regwrite(SPI *spi, uint8_t reg, uint8_t payload){
  spi->write(reg);
  spi->write(payload);
}
