// TinyCharger - Li-Ion Battery Charger based on ATtiny25/45/85
//
// TinyCharger is a simple single cell Li-Ion battery charger with
// selectable charging current and OLED display for charging monitoring.
// The TP4056 is used for charging. The charging current is programmed
// via a resistor, the value of which can be set by the ATtiny via two
// MOSFETs and changed by pressing the SET button. An INA219 continuously
// measures the charging current and voltage and transmits the values to
// the ATtiny via I2C. From this, the ATtiny calculates the state of
// charge and capacity and displays the values on the OLED screen.
//
//                         +-\/-+
// RESET --- A0 (D5) PB5  1|°   |8  Vcc
// PROG1 --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- OLED/INA (SCK)
// PROG2 --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ SET BUTTON
//                   GND  4|    |5  PB0 (D0) ------ OLED/INA (SDA)
//                         +----+
//
// Core:    ATtinyCore (https://github.com/SpenceKonde/ATTinyCore)
// Board:   ATtiny25/45/85 (No bootloader)
// Chip:    ATtiny25 or 45 or 85 (depending on your chip)
// Clock:   8 MHz (internal)
// Millis:  disabled
// B.O.D.:  2.7V
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Note: The internal oscillator may need to be calibrated for precise
//       charging capacity calculation.
//
// The I²C OLED implementation is based on TinyOLEDdemo
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// 2021 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Oscillator calibration value (uncomment and set if necessary)
// #define OSCCAL_VAL  0x48

// Libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// Pin definitions
#define I2C_SDA     PB0     // I2C serial data pin
#define I2C_SCL     PB2     // I2C serial clock pin
#define SETBUTTON   PB1     // SET button pin
#define PROG1       PB3     // Charging current prog pin 1
#define PROG2       PB4     // Charging current prog pin 2

// Current limit values and battery level voltage thresholds
uint16_t progCurrent[4] = {100, 350, 750, 1000};         // current limit values
uint16_t batLevel[5] = {3000, 3600, 4000, 4100, 65535};  // batLevel voltage thresholds

// -----------------------------------------------------------------------------
// I2C Implementation
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRB &= ~(1<<I2C_SDA)   // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRB |=  (1<<I2C_SDA)   // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRB &= ~(1<<I2C_SCL)   // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRB |=  (1<<I2C_SCL)   // SCL as output -> pulled LOW  by MCU
#define I2C_SDA_READ()  (PINB  &  (1<<I2C_SDA)) // read SDA line
#define I2C_DELAY()     asm("lpm")              // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH();I2C_DELAY();I2C_SCL_LOW()  // clock out

// I2C init function
void I2C_init(void) {
  DDRB  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));      // pins as input (HIGH-Z) -> lines released
  PORTB &= ~((1<<I2C_SDA)|(1<<I2C_SCL));      // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i=8; i; i--, data<<=1) {        // transmit 8 bits, MSB first
    (data & 0x80) ? (I2C_SDA_HIGH()) : (I2C_SDA_LOW());  // SDA HIGH if bit is 1
    I2C_CLOCKOUT();                           // clock out -> slave reads the bit
  }
  I2C_DELAY();                                // delay 3 clock cycles
  I2C_SDA_HIGH();                             // release SDA for ACK bit of slave
  I2C_CLOCKOUT();                             // 9th clock pulse is for the ignored ACK bit
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                              // start condition: SDA goes LOW first
  I2C_SCL_LOW();                              // start condition: SCL goes LOW second
  I2C_write(addr);                            // send slave address
}

// I2C restart transmission
void I2C_restart(uint8_t addr) {
  I2C_SDA_HIGH();                             // prepare SDA for HIGH to LOW transition
  I2C_SCL_HIGH();                             // restart condition: clock HIGH
  I2C_start(addr);                            // start again
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                              // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                             // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                             // stop condition: SDA goes HIGH second
}

// I2C receive one data byte from the slave (ack=0 for last byte, ack>0 if more bytes to follow)
uint8_t I2C_read(uint8_t ack) {
  uint8_t data = 0;                           // variable for the received byte
  I2C_SDA_HIGH();                             // release SDA -> will be toggled by slave
  for(uint8_t i=8; i; i--) {                  // receive 8 bits
    data<<=1;                                 // bits shifted in right (MSB first)
    I2C_DELAY();                              // delay 3 clock cycles
    I2C_SCL_HIGH();                           // clock HIGH
    if(I2C_SDA_READ()) data |=1;              // read bit
    I2C_SCL_LOW();                            // clock LOW -> slave prepares next bit
  }
  if (ack) I2C_SDA_LOW();                     // pull SDA LOW to acknowledge (ACK)
  I2C_DELAY();                                // delay 3 clock cycles
  I2C_CLOCKOUT();                             // clock out -> slave reads ACK bit
  return(data);                               // return the received byte
}

// -----------------------------------------------------------------------------
// OLED Implementation
// -----------------------------------------------------------------------------

// OLED definitions
#define OLED_ADDR       0x78    // OLED write address
#define OLED_CMD_MODE   0x00    // set command mode
#define OLED_DAT_MODE   0x40    // set data mode
#define OLED_INIT_LEN   11      // 9: no screen flip, 11: screen flip

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xA8, 0x1F,                   // set multiplex for 128x32
  0x20, 0x01,                   // set vertical memory addressing mode
  0xDA, 0x02,                   // set COM pins hardware configuration to sequential
  0x8D, 0x14,                   // enable charge pump
  0xAF,                         // switch on OLED
  0xA1, 0xC8                    // flip the screen
};

// OLED 5x16 font
const uint8_t OLED_FONT[] PROGMEM = {
  0x7C, 0x1F, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x7C, 0x1F, // 0  0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x1F, // 1  1
  0x00, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x00, // 2  2
  0x00, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 3  3
  0x7C, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7C, 0x1F, // 4  4
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 5  5
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 6  6
  0x0C, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x7C, 0x1F, // 7  7
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 8  8
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 9  9
  0xF0, 0x3F, 0x8C, 0x00, 0x82, 0x00, 0x8C, 0x00, 0xF0, 0x3F, // A 10
  0xFE, 0x3F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x02, 0x20, // E 11
  0x7C, 0x10, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x04, 0x1F, // S 12
  0x02, 0x00, 0x02, 0x00, 0xFE, 0x3F, 0x02, 0x00, 0x02, 0x00, // T 13
  0xFE, 0x07, 0x00, 0x18, 0x00, 0x20, 0x00, 0x18, 0xFE, 0x07, // V 14
  0xFE, 0x3F, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x3F, // h 15
  0x80, 0x3F, 0x80, 0x00, 0x80, 0x3F, 0x80, 0x00, 0x00, 0x3F, // m 16
  0x00, 0x00, 0x30, 0x06, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00, // : 17
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   18
  0xC0, 0x1C, 0xA0, 0x18, 0x90, 0x14, 0x88, 0x02, 0x84, 0x01, // CHRG 19
  0xFC, 0x3F, 0x04, 0x20, 0x06, 0x20, 0x04, 0x20, 0xFC, 0x2F, // BAT0 20
  0xFC, 0x3F, 0x04, 0x38, 0x06, 0x38, 0x04, 0x38, 0xFC, 0x3F, // BAT1 21
  0xFC, 0x3F, 0x04, 0x3F, 0x06, 0x3F, 0x04, 0x3F, 0xFC, 0x3F, // BAT2 22
  0xFC, 0x3F, 0xE4, 0x3F, 0xE6, 0x3F, 0xE4, 0x3F, 0xFC, 0x3F, // BAT3 23
  0xFC, 0x3F, 0xFC, 0x3F, 0xFE, 0x3F, 0xFC, 0x3F, 0xFC, 0x3F  // BAT4 24
};

// OLED init function
void OLED_init(void) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for (uint8_t i=0; i<OLED_INIT_LEN; i++) 
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                             // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(0x22);                        // command for min/max page
  I2C_write(ypos); I2C_write(ypos+1);     // min: ypos; max: ypos+1
  I2C_write(xpos & 0x0F);                 // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));          // set high nibble of start column
  I2C_write(0xB0 | (ypos));               // set start page
  I2C_stop();                             // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_setCursor(0, 0);                   // set cursor at upper half
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  uint8_t i = 0;                          // count variable
  do {I2C_write(0x00);} while (--i);      // clear upper half
  I2C_stop();                             // stop transmission
  OLED_setCursor(0, 2);                   // set cursor at lower half
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  do {I2C_write(0x00);} while (--i);      // clear upper half
  I2C_stop();                             // stop transmission
}

// OLED plot a character
void OLED_plotChar(uint8_t ch) {
  ch = (ch << 1) + (ch << 3);             // calculate position of character in font array
  I2C_write(0x00); I2C_write(0x00);       // print spacing between characters
  for(uint8_t i=10; i; i--) I2C_write(pgm_read_byte(&OLED_FONT[ch++])); // print character
}

// OLED print a character
void OLED_printChar(uint8_t ch) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_plotChar(ch);                      // plot the character
  I2C_stop();                             // stop transmission
}

// OLED print a string from program memory; terminator: 255
void OLED_printString(const uint8_t* p) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  uint8_t ch = pgm_read_byte(p);          // read first character from program memory
  while (ch < 255) {                      // repeat until string terminator
    OLED_plotChar(ch);                    // plot character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED print 16-bit value as 4-digit decimal (BCD conversion by substraction method)
void OLED_printDec4(uint16_t value) {
  static uint16_t divider[5] = {1000, 100, 10, 1};  // for BCD conversion
  if (value > 9999) value = 9999;                   // limit 4-digit value
  I2C_start(OLED_ADDR);                             // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                         // set data mode
  for(uint8_t digit = 0; digit < 4; digit++) {      // 4 digits
    uint8_t digitval = 0;                           // start with digit value 0
    while (value >= divider[digit]) {               // if current divider fits into the value
      digitval++;                                   // increase digit value
      value -= divider[digit];                      // decrease value by divider
    }
    OLED_plotChar(digitval);                        // print the digit
  }
  I2C_stop();                                       // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec2(uint8_t value) {
  if (value > 99) value = 99;                       // limit 2-digit value
  I2C_start(OLED_ADDR);                             // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                         // set data mode
  uint8_t digitval = 0;                             // start with digit value 0
  while (value >= 10) {                             // if current divider fits into the value
    digitval++;                                     // increase digit value
    value -= 10;                                    // decrease value by divider
  }
  OLED_plotChar(digitval);                          // print first digit
  OLED_plotChar(value);                             // print second digit
  I2C_stop();                                       // stop transmission
}

// -----------------------------------------------------------------------------
// INA219 Implementation
// -----------------------------------------------------------------------------

// INA219 register values
#define INA_ADDR        0x80                // I2C write address of INA219
#define INA_CONFIG      0b0000011001100111  // INA config register according to datasheet
#define INA_CALIB       5120                // INA calibration register according to R_SHUNT
#define INA_REG_CONFIG  0x00                // INA configuration register address
#define INA_REG_CALIB   0x05                // INA calibration register address
#define INA_REG_SHUNT   0x01                // INA shunt voltage register address
#define INA_REG_VOLTAGE 0x02                // INA bus voltage register address
#define INA_REG_POWER   0x03                // INA power register address
#define INA_REG_CURRENT 0x04                // INA current register address

// INA219 write a register value
void INA_write(uint8_t reg, uint16_t value) {
  I2C_start(INA_ADDR);                      // start transmission to INA219
  I2C_write(reg);                           // write register address
  I2C_write(value >> 8);                    // write register content high byte
  I2C_write(value);                         // write register content low  byte
  I2C_stop();                               // stop transmission
}

// INA219 read a register
uint16_t INA_read(uint8_t reg) {
  uint16_t result;                          // result variable
  I2C_start(INA_ADDR);                      // start transmission to INA219
  I2C_write(reg);                           // write register address
  I2C_restart(INA_ADDR | 0x01);             // restart for reading
  result = (uint16_t)(I2C_read(1) << 8) | I2C_read(0);  // read register content
  I2C_stop();                               // stop transmission
  return(result);                           // return result
}

// INA219 write inital configuration and calibration values
void INA_init(void) {
  INA_write(INA_REG_CONFIG, INA_CONFIG);    // write INA219 configuration
  INA_write(INA_REG_CALIB,  INA_CALIB);     // write INA219 calibration
}

// INA219 read voltage
uint16_t INA_readVoltage(void) {
  return((INA_read(INA_REG_VOLTAGE) >> 1) & 0xFFFC);
}

// INA219 read sensor values
uint16_t INA_readCurrent(void) {
  uint16_t result =  INA_read(INA_REG_CURRENT);
  if (result > 32767) result = 0;
  if (result < 3)     result = 0;
  return(result);
}

// -----------------------------------------------------------------------------
// Millis Counter Implementation for Timer0
// -----------------------------------------------------------------------------

volatile uint32_t MIL_counter = 0;  // millis counter variable

// Init millis counter
void MIL_init(void) {
  OCR0A  = 124;                     // TOP: 124 = 8000kHz / (64 * 1kHz) - 1
  TCCR0A = (1<<WGM01);              // timer0 CTC mode
  TCCR0B = (1<<CS01)|(1<<CS00);     // start timer0 with prescaler 64
  TIMSK  = (1<<OCIE0A);             // enable output compare match interrupt
  sei();                            // enable global interrupts
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                            // disable interrupt for atomic read
  uint32_t result = MIL_counter;    // read millis counter
  sei();                            // enable interrupts
  return(result);                   // return millis counter value
}

// Timer0 compare match A interrupt service routine (every millisecond)
ISR(TIM0_COMPA_vect) {
  MIL_counter++;                    // increase millis counter
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

// Some "strings"
const uint8_t SET[] PROGMEM = { 18, 12, 11, 13, 17, 255 };  // " SET:"
const uint8_t mA[]  PROGMEM = { 16, 10, 18, 255 };          // "mA "
const uint8_t mV[]  PROGMEM = { 16, 14, 18, 255 };          // "mV "
const uint8_t mAh[] PROGMEM = { 16, 10, 15, 255 };          // "mAh"

int main(void) {
  // Local variables
  uint16_t  voltage, current;                 // voltage in mV, current in mA
  uint32_t  lastmillis, nowmillis, interval;  // timer variables in milliseconds
  uint16_t  seconds;                          // charging time in seconds
  uint32_t  duration   = 0;                   // charging time in milliseconds
  uint32_t  capacity   = 0;                   // charged capacity
  uint8_t   progState  = 0;                   // current limit prog state
  uint8_t   lastbutton = 1;                   // button state    (0: button pressed)
  uint8_t   isCharging;                       // chargin state   (0: not charging)
  uint8_t   batState;                         // state of charge (0..4)

  // Set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                      // set the value if defined above
  #endif
  
  // Setup
  PORTB |= (1<<SETBUTTON);                    // pullup on button pin
  DDRB  |= (1<<PROG1) | (1<<PROG2);           // prog pins as output
  
  I2C_init();                                 // init I2C
  INA_init();                                 // init INA219
  MIL_init();                                 // init millis timer
  OLED_init();                                // init OLED
  OLED_clearScreen();                         // clear screen
  
  lastmillis  = MIL_read();                   // read millis
  
  // Loop
  while(1) {  
    // Read sensors and set state variables
    voltage = INA_readVoltage();              // read voltage from INA219
    current = INA_readCurrent();              // read current from INA219
    isCharging  = (current > 5);              // set charging state variable
    batState = 0;                             // set state of charge ...
    while(voltage > batLevel[batState]) batState++;
    if(isCharging && (batState == 4)) batState = 3;
    
    // Timing stuff
    nowmillis   = MIL_read();                 // read millis
    interval    = nowmillis - lastmillis;     // calculate time interval
    lastmillis  = nowmillis;                  // update lastmillis
    if (isCharging) {                         // if in charging mode:
      duration += interval;                   // calculate total charging duration
      capacity += interval * current / 3600;  // calculate uAh
    }
    seconds     = (uint32_t)duration / 1000;  // calculate total charging time in seconds   
    if (seconds > 35999) seconds = 35999;     // limit seconds timer
 
    // Check button and set charging current limit accordingly (blocked in charging mode)
    if (PINB & (1<<SETBUTTON)) lastbutton = 0;
    else if (!lastbutton) {
      if (!isCharging) {
        if (++progState > 3) progState = 0;
        (progState & 0x01) ? (PORTB |= (1<<PROG1)) : (PORTB &= ~(1<<PROG1));
        (progState & 0x02) ? (PORTB |= (1<<PROG2)) : (PORTB &= ~(1<<PROG2));
      }
      lastbutton  = 1;
    }
    
    // Update OLED
    OLED_setCursor(0,0);
    OLED_printChar(batState + 20);
    (isCharging) ? (OLED_printChar(19)) : (OLED_printChar(18));
    OLED_printString(SET);
    OLED_printDec4(progCurrent[progState]);
    OLED_printString(mA);
    OLED_printChar(seconds / 3600);
    OLED_printChar(17);
    OLED_printDec2((seconds % 3600) / 60);
    OLED_printChar(17);
    OLED_printDec2(seconds % 60);
    
    OLED_setCursor(0,2);
    OLED_printDec4(voltage);
    OLED_printString(mV);
    OLED_printDec4(current);
    OLED_printString(mA);
    OLED_printDec4(capacity / 1000);
    OLED_printString(mAh);
    
    _delay_ms(100);
  }
}
