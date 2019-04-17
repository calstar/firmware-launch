#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "MPL3115A2.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"

#include "mbed.h"
#include "pins.h"

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

using namespace Calstar;

DigitalOut led_red(STATE_LED_RED);
DigitalOut led_green(STATE_LED_GREEN);
DigitalOut led_blue(STATE_LED_BLUE);

Serial debug_uart(DEBUG_TX, DEBUG_RX, DEBUG_UART_BAUDRATE);

I2C mpl_i2c(I2C_SENSOR_SDA, I2C_SENSOR_SCL);
MPL3115A2 alt(&mpl_i2c, &debug_uart);

SPI kx222(KX222_MOSI, KX222_MISO, KX222_SCLK, KX222_CS);
//DigitalOut kx222_cs(KX222_CS);

SDBlockDevice sd1(SD_MOSI, SD_MISO, SD_SCLK, SD1_CS);
SDBlockDevice sd2(SD_MOSI, SD_MISO, SD_SCLK, SD2_CS);

FATFileSystem fs1("fs1");
FATFileSystem fs2("fs2");

uint8_t regread(SPI *spi, uint8_t reg);
void regwrite(SPI *spi, uint8_t reg, uint8_t payload);

time_t read_rtc(void) {
    return 0;
}

int get_flight(){
  FILE *a = fopen("/fs1/flightindex.txt", "r+");
  FILE *b = fopen("/fs2/flightindex.txt", "r+");
  int amax = 0, bmax = 0, moose;

  if(a){
    fscanf(a, "%d", &amax);
    fclose(a);
  }
  if(b){
    fscanf(b, "%d", &bmax);
    fclose(b);
  }
  a = fopen("/fs1/flightindex.txt", "w+");
  b = fopen("/fs2/flightindex.txt", "w+");
  moose = bmax > amax ? bmax: amax;
  fprintf(a, "%d\n", moose + 1);
  fprintf(b, "%d\n", moose + 1);
  fflush(a);
  fflush(b);
  fclose(a);
  fclose(b);

  return moose + 1;
}

int main() {
  uint8_t altimiter = alt.whoAmI();
  uint8_t kxwhoami;
  uint8_t led_green = 1;
  int flight_num;
  char file_name_bufa[100], file_name_bufb[100];
  FILE *outa, *outb;

  attach_rtc(&read_rtc, NULL, NULL, NULL);

  led_red = 1;
  led_blue = 1;

  debug_uart.printf("---- CalSTAR BLACK BOX ----\r\n");
  debug_uart.printf("Mounting FS1:\r\n");
  debug_uart.printf("%s\r\n", (fs1.mount(&sd1) ? "Fail :(" : "OK"));

  debug_uart.printf("Mounting FS2:\r\n");
  debug_uart.printf("%s\r\n", (fs2.mount(&sd2) ? "Fail :(" : "OK"));

  flight_num = get_flight();

  snprintf(file_name_bufa, 100, "/fs1/%d.csv", flight_num);
  snprintf(file_name_bufb, 100, "/fs2/%d.csv", flight_num);

  debug_uart.printf("Opening: %s\r\n", file_name_bufa);
  outa = fopen(file_name_bufa, "w+");
  if(!outa){
    debug_uart.printf("Open Failed: %s\r\n", file_name_bufa);
  }

  debug_uart.printf("Opening: %s\r\n", file_name_bufb);
  outb = fopen(file_name_bufb, "w+");
  if(!outa){
    debug_uart.printf("Open Failed: %s\r\n", file_name_bufb);
  }
  
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
