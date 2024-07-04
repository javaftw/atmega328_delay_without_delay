/******************************************************************************/
/*                             PROJECT PINOUT                                 */
/******************************************************************************/
/*                             ATMega328P
 *                                ______
 *            RESET/PCINT14/PC6 =|01* 28|= PC5/PCINT13/SCL/ADC5
 *               RX/PCINT16/PD0 =|02  27|= PC4/PCINT12/SDA/ADC4
 *               TX/PCINT17/PD1 =|03  26|= PC3/PCINT11/ADC3
 *             INT0/PCINT18/PD2 =|04  25|= PC2/PCINT10/ADC2
 *                  PCINT19/PD3 =|05  24|= PC1/PCINT9/ADC1
 *                  PCINT20/PD4 =|06  23|= PC0/PCINT8/ADC0
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *             XTAL1/PCINT6/PB6 =|09  20|= AVcc
 *             XTAL2/PCINT7/PB7 =|10  19|= PB5/PCINT5/SCK
 *             OC0B/PCINT21/PD5 =|11  18|= PB4/PCINT4/MISO
 *        OC0A/AIN0/PCINT22/PD6 =|12  17|= PB3/PCINT3/MOSI/OC2A/OC2
 *             AIN1/PCINT23/PD7 =|13  16|= PB2/PCINT2/SS/OC1B
 *                   PCINT0/PB0 =|14  15|= PB1/PCINT1/OC1A
 *                                ------
 * 
 *                                ______
 *                              =|01* 28|= SCL (LCD)
 *                           RX =|02  27|= SDA (LCD)
 *                           TX =|03  26|= 
 *                              =|04  25|= Sensor_X
 *                              =|05  24|= Sensor_Y
 *                              =|06  23|= Sensor_Z
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *                              =|09  20|= 
 *                              =|10  19|= 
 *                              =|11  18|= 
 *                     LED_R(X) =|12  17|= 
 *                     LED_G(Y) =|13  16|= 
 *                     LED_B(Z) =|14  15|= BTN
 *                                ------
 * 
 * 
 */

/******************************************************************************/
/*                                   DEFS                                     */
/******************************************************************************/

#define F_CPU   8000000
//#define BAUD   19200
#define BAUD   38400
#define MEMSIZE   2048
#define TRUE    1
#define FALSE   0
#define ON      1
#define OFF     0
#define HIGH    1
#define LOW     0
#define NOP     asm("nop")
#define TX_BUF_LEN  8
#define RX_BUF_LEN  8

//---MACROS
#define CMD_EQ(A)       strcmp(A, rxbuf) == 0
#define CMD_EQN(A,N)    strncmp(A, rxbuf, N) == 0
#define ITOA(A)         itoa(A, txbuf, 10)
#define ITOA2(A)         itoa(A, txbuf, 2)
#define ITOA16(A)         itoa(A, txbuf, 16)

#define _testpinb(a)         if((PINB & a) == a)
#define _setpinb(p)          PORTB |= p
#define _clearpinb(p)        PORTB &= ~(p)
#define _togglepinb(p)       PORTB ^= p

/******************************************************************************/
/*                                 INCLUDES                                   */
/******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <util/twi.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/*                                VARIABLES                                   */

/******************************************************************************/

char txbuf[TX_BUF_LEN];
char rxbuf[RX_BUF_LEN];
unsigned char reclen = 0;
uint32_t timer_counter;

const char str_00[] PROGMEM = "ATMega329P Tests and Experiments ";
const char str_01[] PROGMEM = "Firmware version: 0.01 ";
const char str_02[] PROGMEM = "";
const char str_03[] PROGMEM = "OK";
const char str_04[] PROGMEM = "FAIL";
const char str_05[] PROGMEM = " used,";
const char str_06[] PROGMEM = " available";


PGM_P const strings[] PROGMEM = {
    str_00,
    str_01,
    str_02,
    str_03,
    str_04,
    str_05,
    str_06
};

/******************************************************************************/
/*                             STRUCTS AND ENUMS                              */

/******************************************************************************/

struct {
    uint16_t IS_RUNNING : 1;
    uint16_t RX_COMPLETE : 1;
uint16_t:
    0; //padding
} state_flags;

struct Commands {
    char *stat = "stat";
    char *pic = "pic";
    char *q = "q";
} cmd;

/******************************************************************************/
/*                            FUNCTION DECLARATIONS                           */
/******************************************************************************/

int main(void);
static void init(void);
void mainloop(void);
//
void handle_rxtx(void);
void handle_running(void);
//general 
void drawPic(void);
//usart
ISR(USART_RX_vect);
void usart_tx(uint8_t c);
void usart_rx(void);
void send_string(char* str);
void send_pgm_string(PGM_P str);
void send_text(char* str);
void send_pgm_text(PGM_P str);
void clear_txbuf(void);
void clear_rxbuf(void);
//i2c
uint8_t i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write_address(uint8_t address);
uint8_t i2c_write_byte(uint8_t b);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nak(void);
void i2c_send_command(uint8_t addr, uint8_t cmd);
//ADC
uint16_t read_adc(uint8_t chan);
uint16_t compute_average(uint16_t data[], uint8_t len);
//eeprom
void eeprom_w_byte(uint8_t address, uint8_t data);
uint8_t eeprom_r_byte(uint8_t address);
//timer
ISR(TIMER0_OVF_vect);
//util
void delay_n_us(uint16_t n);
void delay_n_ms(uint16_t n);
int available_sram(void);

/******************************************************************************
 *                                FUNCTIONS                                   *
 ******************************************************************************/

int main(void) {
    //variable initialization
    state_flags.IS_RUNNING = TRUE;
    //function initialization
    init();
    mainloop();
    return 0;
}

/*******************************************************************************
 *                                                                        INIT*/

static void init(void) {

    //=======================================================================I/O
    //Direction registers port c 1=output 0=input
    /*If DDxn is written logic one, Pxn is configured as an output pin. 
     * If DDxn is written logic zero, Pxn is configured as an input pin.
     * If PORTxn is written logic one AND the pin is configured as an input pin (0), 
     * the pull-up resistor is activated.
     */
    DDRB |= 0b00000001; //PC0 is input
    PORTB |= 0b00000010; //Pull-up activated on PC0

    DDRC |= 0b00000000; //
    PORTC |= 0b11111111; //Pull-up activated on PC0

    DDRD |= 0b11000000; //
    PORTD |= 0b00000000; //Pull-up activated on PC0

    //=======================================================================ADC
    ADMUX = 0x00; // | _BV(MUX0);//source is AVCC, select only channel 1
    //enable ADC, select ADC clock to 8000000Hz/64 = 125000Hz (ADPS = 110)
    ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);

    //======================================================================UART
    UBRR0H = UBRRH_VALUE; //(_UBRR)>>8;
    UBRR0L = UBRRL_VALUE; //_UBRR;
    //enable rx and tx and rx interrupt
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
    //frame format 8 data 1 stop
    UCSR0C |= _BV(UCSZ00) | _BV(UCSZ01);

    //====================================================================TIMER0
    TCNT0 = 0x00;
    TCCR0B |= _BV(CS00); // | (1 << CS02);//prescaler
    TIMSK0 |= _BV(TOV0); //timer 0 overflow interrupt enable

    //=======================================================================I2C
    //SCL period = F_CPU/(16 + 2(TWBR.TWI) * (TWSR.TWI)) 
    //where TWSR.TWI is the prescaler value and TWBR.TWI is the bit rate
    TWBR = 0x05; //bit rate
    TWSR |= _BV(TWPS1) /*| _BV(TWPS0)*/; // Prescaler 00=1, 01=4, 10=16, 11=64
    //TWCR |= _BV(TWEN); // Enable TWI

    //================================================================INTERRUPTS
    sei();
}

/******************************************************************************
 *                                                                    MAINLOOP*/
void mainloop(void) {

    send_pgm_text(strings[0]);
    send_pgm_text(strings[1]);
    send_pgm_string(strings[3]);
    while (TRUE) {
        if (state_flags.RX_COMPLETE) {
            handle_rxtx();
        }
        if (state_flags.IS_RUNNING) {
            handle_running();
        }
    }
    return;
}

/******************************************************************************
 *                                                                STATE HANDLERS*/

void handle_rxtx(void) {
    if (CMD_EQ(cmd.stat)) {
        int avram = available_sram();
        send_text(ITOA(MEMSIZE - avram));
        send_pgm_text(strings[5]);
        send_text(ITOA(avram));
        send_pgm_string(strings[6]);
    } else if (CMD_EQ(cmd.pic)) {
        drawPic();
    } else if (CMD_EQ(cmd.q)) {
        char x = 1;
        char *px;
        px = &x;
        int y = 1;
        int *py;
        py = &y;
        int apx = (int)&x;
        int apy = (int)&y;
        send_string(ITOA16(apx));
        send_string(ITOA16(apy));
    }
    clear_rxbuf();
    state_flags.RX_COMPLETE = FALSE;
    return;
}

void handle_running(void) {
    NOP;
    return;
}

/******************************************************************************
 *                                                           GENERAL FUNCTIONS*/

void drawPic(void) {
    uint16_t num = 0;
    for (uint8_t i = 0; i < 36; i++) {
        for (uint8_t j = 0; j < 36; j++) {
            num = i*j;
            if (num %13 == 0) {
                send_text("*");
            } else {
                send_text("-");
            }
        }
        send_string(" ");
    }
    return;
}



/******************************************************************************
 *                                                                       USART*/

void usart_tx(uint8_t c) {
    //count++;
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void usart_rx(void) {
    while (!(UCSR0A & (1 << RXC0)));
    volatile unsigned char c = UDR0;
    if (reclen == (RX_BUF_LEN + 1) || c == 0x0D || c == 0x0A || c == 0x00) {
        state_flags.RX_COMPLETE = TRUE;
    } else if (!state_flags.RX_COMPLETE) {
        rxbuf[reclen] = c;
        reclen++;
    }
}

void send_string(char str[]) {
    for (uint8_t idx = 0; str[idx] != '\0'; idx++) {
        usart_tx(str[idx]);
    }
    usart_tx(0x0A);
}

void send_pgm_string(PGM_P str) {
    char c;
    while ((c = pgm_read_byte(str++)) != 0) {
        usart_tx(c);
    }
    usart_tx(0x0A);
}

void send_text(char str[]) {
    for (uint8_t idx = 0; str[idx] != '\0'; idx++) {
        usart_tx(str[idx]);
    }
}

void send_pgm_text(PGM_P str) {
    char c;
    while ((c = pgm_read_byte(str++)) != 0) {
        usart_tx(c);
    }
}

void clear_txbuf(void) {
    for (uint8_t idx = 0; txbuf[idx] != '\0';) {
        txbuf[idx] = 0x00;
        idx++;
    }
}

void clear_rxbuf(void) {
    for (uint8_t idx = 0; rxbuf[idx] != '\0';) {
        rxbuf[idx] = 0x00;
        idx++;
    }
    reclen = 0;
}

ISR(USART_RX_vect) {
    usart_rx();
}

/*****************************************************************************
 *                                                                     I2C LCD*/


uint8_t i2c_start(void) {
    //generate I2C start condition
    //START=SDA transitions from HI to LO while SCL is HI
    //    _
    //SDA  \__
    //    __
    //SCL   \_
    TWCR = 0;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Send start condition
    while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the START condition has been transmitted
    if ((TWSR & 0xF8) != TW_START) return 1;
    return 0;
}

void i2c_stop(void) {
    //generate I2C stop condition
    //STOP=SDA transitions from LO to HI while SCL is HI
    //       _
    //SDA __/
    //      __
    //SCL _/
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Transmit STOP condition
}

uint8_t i2c_write_address(uint8_t address) {
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
    uint8_t twsr = TWSR & 0xF8;
    if ((twsr & TW_MT_SLA_ACK) != TW_MT_SLA_ACK && (twsr & TW_MR_SLA_ACK) != TW_MR_SLA_ACK) return 1;
    return 0;
}

uint8_t i2c_write_byte(uint8_t b) {
    TWDR = b;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;
    return 0;
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t i2c_read_nak(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void i2c_send_command(uint8_t addr, uint8_t cmd) {
    uint8_t cmnd;
    cmnd = cmd;
    volatile uint8_t a;
    volatile uint8_t c;
    c = 0xff;
    if (i2c_start() != 0) goto ERROR_START;
    a = i2c_write_address(addr);
    if (a != 0) goto ERROR_ADDR;
    if (i2c_read_ack() != addr) goto ERROR_ADDR;
    if (i2c_write_byte(cmnd) != 0) goto ERROR_CMD;
    c = i2c_read_ack();
    if (c != cmnd) goto ERROR_CMD;
    i2c_stop();
    return;

ERROR_START:{
        send_string("ERR_START");
    }

ERROR_ADDR:{
        send_text("ERR_ADDR addr:");
        send_text(ITOA16(addr));
        send_text(" got:");
        send_string(ITOA16(a));
        return;
    }

ERROR_CMD:{
        send_text("ERR_CMD cmd:");
        send_text(ITOA16(cmnd));
        send_text(" got:");
        send_string(ITOA16(a));
        return;
    }
}

/******************************************************************************
 *                                                                        ADC*/

uint16_t read_adc(uint8_t chan) {
    ADMUX = chan; //channel 1
    _delay_ms(1);
    ADCSRA |= _BV(ADSC); //enable adc and start conversion
    while (ADCSRA & _BV(ADSC)); //wait for conversion to complete
    return ADCW;
}

/******************************************************************************
 *                                                                      TIMER0*/
ISR(TIMER0_OVF_vect) {
    timer_counter++;
    if (timer_counter > 0x7a12) {
        timer_counter = 0;
    }
}

/*****************************************************************************
 *                                                                      EEPROM*/

void eeprom_w_byte(uint8_t address, uint8_t data) {
    //const uint8_t addr = address;
    eeprom_busy_wait();
    eeprom_write_byte((uint8_t *) address, data);
}

uint8_t eeprom_r_byte(uint8_t address) {
    //const uint8_t *addr = address;
    //const uint8_t *addr2 = address++;
    eeprom_busy_wait();
    uint8_t ret = eeprom_read_byte((uint8_t *) address);
    return ret;
}

/*****************************************************************************
 *                                                                        UTIL*/

void delay_n_us(uint16_t n) {
    while (n--) {
        _delay_us(1);
    }
}

void delay_n_ms(uint16_t n) {
    while (n--) {
        _delay_ms(1);
    }
}

int available_sram(void) {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}