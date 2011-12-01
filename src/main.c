#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>

#define IRPORT PINB
#define IRPIN 3
#define IRTIMER TCNT0

/*
  Set the timer so it overflows in 16.384ms (space between messages is 39ms)
  that means we are capturing time with 0.064ms resolution (15.625 per ms)

  Protocol of Leadtek WinFast Y040052 remote

  Signal Length CNT Meaning

  0      >10ms  OVF timer overflow while at 0 means error (sensor disconnected?)
  0      9 ms   229 First in the message, announces new message
  0      0.64ms  16 Ends a bit, get the length of preceeding 1

  1      >10ms  OVF timer overflow while at 1 means end of the message
  1      4.48ms 114 If it is the first bit, then it is start bit 
  1      2.1ms   53 If it is the first bit, then it is also the last and denotes repetition of previous message
  1      1.6ms   40 1
  1      0.48ms  12 0
*/

#define REMOTE_ID 0x03

#define START_BIT_L 50 //3.2ms @ 1Mhz, 64 prescale
#define REPEAT_L 31 //1.984ms
#define ONE_BIT_L 16 //1.024ms
#define PREAMBLE_L 93 //5.952ms

typedef enum{A_ERROR = 0xfe, A_NONE = 0xff,
             A_PWR = 0x00,
             A_B1  = 0x05,
             A_B2  = 0x06
} action_t;

void shiftout(uint8_t data);

action_t processIR(volatile uint8_t msg[4])
{

    /*
    shiftout(msg[0]);
    shiftout(msg[1]);
    shiftout(msg[2]);
    shiftout(msg[3]);
    */

    /* check the inverted bytes for errors */
    if (((msg[0] ^ msg[1]) != 0xff) || 
        ((msg[2] ^ msg[3]) != 0xff)) return A_ERROR;

    /* check remote id for match */
    if (msg[0] != REMOTE_ID) return A_NONE;

    /* return action/button */
    return (action_t)(msg[2]);
}

volatile uint8_t bit_c = 0; // bit no. we are about to receive (255 denotes repeated command, 254 not ready state)
volatile uint8_t data[4];
volatile enum{D_NOTHING = 0,
              D_PREAMBLE,
              D_RECEIVING_NEEDACK,
              D_RECEIVING,
              D_ALLREAD,
              D_DATA,
              D_REPEAT} state = D_NOTHING;

void setup(void)
{
    /* disable interrupts */
    cli();

    /* set pin directions */
    DDRB = _BV(PB1) | _BV(PB4); // all inputs except MISO and PB4
    PORTB = 0x00; // no pull-ups and default output 0

    /* disable ADC */
    ACSR |= _BV(ACD);

    /* enable change interrupts */
    GIMSK |= _BV(PCIE);
    PCMSK = _BV(PCINT3);

    /* setup timer0 including it's interrupt */
    TCCR0A = 0x00;
    TCCR0B = _BV(CS01) | _BV(CS00); // normal mode 0 - 0xff, prescale by 64
    TIMSK |= _BV(TOIE0); // enable overflow interrupt

    /* setup SPI slave mode with external clock on positive edge */
    USICR = _BV(USIWM0) | _BV(USICS1);

    /* reenable interrupts */
    sei();
}

int main(void)
{
    action_t action;

    setup();

    while(1) {
        if (state == D_DATA || state == D_REPEAT) {
            action = processIR(data);
            state = D_NOTHING;

            /* put the last command to the data buffer for SPI */
            USIDR = action;
            
            /* process commands */
            switch (action) {
            default: break;
            }
        }

        /* we should introduce some sleep state here..*/
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
}

/*
  shifts one byte MSB first on PB4
  as H pulses of different length separated by L
  1 is longer, 0 is shorter
*/
void shiftout(uint8_t data)
{
    uint8_t i;
    PORTB &= ~_BV(PB4); //start bit

    for (i=0; i<8; i++) {
        if (data & 0x80) {
            PORTB |= _BV(PB4);
        }
        else {
            PORTB &= ~_BV(PB4);
        }

        PORTB |= _BV(PB4);
        PINB |= _BV(PB4);

        data <<= 1;
    }
}

/* Pin change interrupt - reading IR data */
ISR(PCINT0_vect) {
    static uint8_t oldbit = _BV(IRPIN); // to ensure change has happened

    // not an IR interrupt
    uint8_t bit = IRPORT & _BV(IRPIN); // current IR pin state
    if (oldbit == bit) return;

    // reset IR timer
    uint8_t timer = TCNT0;
    TCNT0 = 0;
    TIFR |= _BV(TOV0); // reset overflow interrupt flag

    // record the change
    oldbit = bit;

    if (state >= D_ALLREAD) return; // we only care about first 32 bits and only if we are ready

    // check data
    if (!bit){ // we are currently low so high bit duration was measured 
        if (timer > START_BIT_L && state == D_PREAMBLE) { // start bit
            bit_c = 0;
            state = D_RECEIVING_NEEDACK;
        }
        else if (timer > REPEAT_L && state == D_PREAMBLE) bit_c = 255; // repetition flag
        else if (timer > ONE_BIT_L && state == D_RECEIVING) {
            data[bit_c >> 3] = (data[bit_c >> 3] >> 1) + 0x80; // shift a log. 1, bit_c >> 3 is the byte number
        }
        else if(state == D_RECEIVING) {
            data[bit_c >> 3] >>= 1; // shift a log. 0
        }
    }
    else{ // low bit timed
        if (timer > PREAMBLE_L && state == D_NOTHING) state = D_PREAMBLE; // preamble low bit (9ms)
        else if (state == D_RECEIVING) {
            if (bit_c == 255) state = D_ALLREAD; // repetition has only one data bit
            else {
                bit_c++; // bit marker
                if (bit_c == 32) state = D_ALLREAD; // all 32 bits received
            }
        }
        else if (state == D_RECEIVING_NEEDACK) state = D_RECEIVING;
    }
}

ISR(TIMER0_OVF_vect) {
    if(state == D_ALLREAD){
        if(bit_c == 255) state = D_REPEAT; // special flag set, this requests repetition of previous command
        else state = D_DATA; // data received
    }
    else state = D_NOTHING; // timeout, reset the state
}
