//#define F_CPU 8000000UL

#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <avr/sleep.h>
#include "ssd1306xled/ssd1306xled.h"
#include "ssd1306xled/font8x16.h"
#include "../usitwix/usitwix/usitwim.h"

#define MLX90614_I2CADDR 0x5A
#define MLX90614_TOBJ1 0x07
#define TEMP_TO_INTREP(x) ( 50*x + 13657 )
#define TESTING_DELAY 100


#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit

void setup_watchdog() 
{
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  
  WDTCR |= (1<<WDCE) | (1<< WDE);
  
  //WDTCR = 6; // 1 sec
  //WDTCR = (1<<WDP3); // 4 sec
  WDTCR = (1<<WDP2) | (1<<WDP0); // 0.5 sec
  sbi(WDTCR, WDIE);
}


// system wakes up when watchdog is timed out
void system_sleep() 
{
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  setup_watchdog();                   // approximately 8 seconds sleep
 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sei();                               // Enable the Interrupts so the wdt can wake us up

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}


extern "C" void WDT_vect(void) __attribute__ ((signal));
void WDT_vect(void) {}


extern "C" void TIM0_COMPA_vect(void) __attribute__ ((signal));
void TIM0_COMPA_vect(void) {
  //PORTB ^= (1 << PB1);
}
void start_alarm() {
  sbi(DDRB, PB1);

  // fast pwm
  sbi(TCCR0B, WGM02);
  sbi(TCCR0A, WGM01);
  sbi(TCCR0A, WGM00);

  sbi(TCCR0A, COM0B1);
  sbi(TCCR0B, CS01);
  OCR0A = 220;
  OCR0B = 110;
  //sbi(TIMSK, OCIE0B);
  

  /*sbi(TCCR0A, WGM00);
  sbi(TCCR0B, WGM02);
  sbi(TCCR0B, CS01);
   // compare register A at 103 us
  OCR0A = 113;
  // interrupt on compare match OCROA == TCNT0
  sbi(TIMSK, OCIE0A);
  sei();*/

}

void ssd1306_string_font8x16xy(uint8_t x, uint8_t y, const char s[], uint8_t len) {
  uint8_t ch, j = 0;
  while (j < len) {
    ch = s[j] - 32; // Convert ASCII code to font data index.
    if (x > 120) {
      x = 0;    // Go to the next line.
      y++;
    }
    ssd1306_setpos(x, y);
    ssd1306_send_data_start();
    for (uint8_t i = 0; i < 8; i++) {
      ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8x16data[ch * 16 + i]));
    }
    ssd1306_send_data_stop();

    ssd1306_setpos(x, y + 1);
    ssd1306_send_data_start();
    for (uint8_t i = 0; i < 8; i++) {
      ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8x16data[ch * 16 + i + 8]));
    }
    ssd1306_send_data_stop();

    x += 8;
    j++;
  }
}

uint16_t read_volt() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(MUX3) | _BV(MUX2);
  
  _delay_ms(2); // Wait for Vref to settle

  ADCSRA |= _BV(ADEN);
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint16_t result = ADCW;
  ADCSRA &= ~_BV(ADEN);
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

bool wasHot = false;
void setup() {
  usitwim_init();
  ssd1306_init();

  cli();
  ssd1306_clear();
  ssd1306_setpos(0,0);
  for (uint16_t i = 0; i < 128 * 9; i++) {
    ssd1306_byte(0x00);
  }
  sei();
}

const uint16_t set_temp = 5500;
void loop() {

    uint8_t i2c_receive_buffer[4];
    uint8_t i2c_receive_buffer_len = 4;

    i2c_receive_buffer[1] = MLX90614_TOBJ1;  //Internal address'
    cli();
    //usitwim_init();
    //_delay_ms(20);
    usitwim_mem_read(MLX90614_I2CADDR, i2c_receive_buffer, i2c_receive_buffer_len);
    sei();

    uint16_t ret = 0;
    ret = i2c_receive_buffer[1];
    ret |= ((uint16_t)i2c_receive_buffer[2]) << 8;
    uint16_t tempx100 = ret*2-27315;
    //float temp = ret*.02 - 273.15;

    if (wasHot && tempx100 < set_temp - 100) { start_alarm(); }
    if (tempx100 > set_temp) wasHot = true;

    char buffer[20];
    itoa(tempx100, buffer, 10);
    cli();
    ssd1306_string_font8x16xy(0, 0, "temp:", 5);
    ssd1306_string_font8x16xy(5*8, 0, buffer, 2);
    ssd1306_string_font8x16xy(7*8, 0, ".", 1);
    ssd1306_string_font8x16xy(8*8, 0, buffer+2, 2);
    ssd1306_string_font8x16xy(12*8, 0, "C", 1);
    sei();

    uint16_t res = read_volt();
    utoa(res, buffer, 10);

    cli();
    ssd1306_string_font8x16xy(0, 2, "batt:", 5);
    ssd1306_string_font8x16xy(5*8, 2, buffer, strlen(buffer));
    ssd1306_string_font8x16xy(12*8, 2, "mV", 2);
    sei();
    
    //  _delay_ms(500);
    system_sleep();
}
int main ()
{
  setup();
  while (1) {loop();}
}