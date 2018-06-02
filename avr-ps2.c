#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD * 16UL) - 1)

#define BUFSIZE 64
#define _CIRBUF_MASK (BUFSIZE - 1)

typedef struct {
  uint8_t buffer[BUFSIZE];
  uint8_t head;
  uint8_t tail;
} cirbuf_t;

static cirbuf_t tx, rx;

static void _cirbuf_init(cirbuf_t *c)
{
  c->head = 0;
  c->tail = 0;
}

static int8_t _cirbuf_push(cirbuf_t *c, uint8_t data)
{
  uint8_t next = (c->head + 1) & _CIRBUF_MASK;
  if (next == c->tail)
    return -1;
  c->buffer[c->head] = data;
  c->head = next;
  return 0;
}

#if 0
static int8_t _cirbuf_pop(cirbuf_t *c, uint8_t *data)
{
  if (c->head == c->tail)
    return -1;
  *data = c->buffer[c->tail];
  c->tail = (c->tail + 1) & _CIRBUF_MASK;
  return 0;
}
#endif
 
ISR(USART1_RX_vect)
{
  uint8_t c;
  c = UDR1;
  _cirbuf_push(&rx, c);
}

ISR(USART1_UDRE_vect)
{
  if (tx.head == tx.tail)
  {
    UCSR1B &= ~_BV(UDRIE1);
    return;
  }
  UDR1 = tx.buffer[tx.tail];
  tx.tail = (tx.tail + 1) & _CIRBUF_MASK;
}

void serial_init(void)
{
  UBRR1H = (BAUDRATE >> 8) & 0xff;
  UBRR1L = (BAUDRATE & 0xff);
  UCSR1B = _BV(TXEN1) | _BV(RXEN1) | _BV(RXCIE1);
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); /* 8-bit */
  _cirbuf_init(&tx);
  _cirbuf_init(&rx);
}

void serial_setup(void)
{
  DDRD |= ~_BV(PD3); /* TX1 */
  PORTD &= ~_BV(PD3);
}

uint8_t hex(uint8_t v) {
  if (v < 10)
    return v + '0';
  return v - 10 + 'a';
}

/*

  PS2 interrupt driven driver

*/

/*

  Documentation source:

  http://www.hardwarebook.info/AT_Keyboard/Mouse_protocol
  http://retired.beyondlogic.org/keyboard/keybrd.htm

*/

#define PS2_CLOCK PD0
#define PS2_CLOCK_DDR DDRD
#define PS2_CLOCK_PORT PORTD
#define PS2_CLOCK_PIN PIND

#define PS2_DATA PE6
#define PS2_DATA_DDR DDRE
#define PS2_DATA_PORT PORTE
#define PS2_DATA_PIN PINE

static volatile struct ps2data {
  uint8_t parity; /* Parity bit */
  uint8_t buf[2];
  uint8_t b; /* Current byte: 0 or 1 */
  uint8_t i; /* Bit index in current byte */
} data = { 0 };

#define PS2_STATE_IDLE 0
#define PS2_STATE_SENDING 1
#define PS2_STATE_SENT 2
#define PS2_STATE_RECEIVING 3
#define PS2_STATE_RECEIVED 4
#define PS2_STATE_NACK 5
#define PS2_STATE_ACK 6

static volatile uint8_t ps2state = PS2_STATE_IDLE;
static volatile uint8_t ps2_rx;

static inline void ps2data_low(void)
{
  PS2_DATA_PORT &= ~_BV(PS2_DATA);
  PS2_DATA_DDR |= _BV(PS2_DATA);
}

static inline void ps2data_high(void)
{
  PS2_DATA_DDR &= ~_BV(PS2_DATA);
  PS2_DATA_PORT |= _BV(PS2_DATA);
}

static inline void ps2clock_low(void)
{
  PS2_CLOCK_PORT &= ~_BV(PS2_CLOCK);
  PS2_CLOCK_DDR |= _BV(PS2_CLOCK);
}

static inline void ps2clock_high(void)
{
  PS2_CLOCK_DDR &= ~_BV(PS2_CLOCK);
  PS2_CLOCK_PORT |= _BV(PS2_CLOCK);
}

static inline void timer_off(void)
{
  TCCR0B = 0;
  ps2state = PS2_STATE_IDLE;
  data.i = 0;
  ps2data_high();
}

static inline void timer_on(void)
{
  TCNT0 = 0;
  TCCR0B |= _BV(CS02) | _BV(CS00);
}

static void timer_init(void)
{
  TCCR0A |= _BV(WGM01);
  OCR0A = 48;
  TIMSK0 = _BV(OCIE0A);
}

static void bit_send(void)
{
  uint8_t bit;

  if (!data.i)
  {
    /* Start bit: stay low */
    data.parity = 0;
    data.b = 0;
    data.i = 1;
    return;
  }
  if (data.buf[data.b] & data.i)
  {
    ps2data_high();
    data.parity = !data.parity;
  }
  else
  {
    ps2data_low();
  }
  data.i <<= 1;
  if (!data.i)
  {
    data.b = 1;
    data.i = 1;
    data.buf[1] = (!data.parity) | 0x06; /* |_ACK_|_STOP_|_PARTIY_| */
  }
  if (data.b && data.i == 0x08)
  {
    /* Stop bit */
    bit = (PS2_DATA_PIN & _BV(PS2_DATA)) == _BV(PS2_DATA);
    if (bit)
    {
      ps2state = PS2_STATE_NACK;
    }
    else
    {
      ps2state = PS2_STATE_SENT;
    }
    data.i = 0;
  }
}

static void bit_receive(void)
{
  uint8_t bit;

  bit = (PS2_DATA_PIN & _BV(PS2_DATA)) == _BV(PS2_DATA);

  if (!bit && !data.i)
  {
    /* Start bit */
    data.parity = 0;
    data.buf[0] = 0;
    data.b = 0;
    data.i = 1;
    ps2_rx = 0;
    ps2state = PS2_STATE_RECEIVING;
    return;
  }

  if (bit)
  {
    data.buf[data.b] |= data.i;
    data.parity = !data.parity;
  }
  data.i <<= 1;
  if (!data.i)
  {
    data.b = 1;
    data.i = 1;
    data.buf[1] = 0;
  }
  else if (data.b && data.i == 0x04)
  {
    timer_off();
    /* Stop bit */
    if (bit && !data.parity)
    {
      if (data.buf[0] == 0xfa)
      {
        ps2state = PS2_STATE_ACK;
      }
      else
      {
        ps2state = PS2_STATE_RECEIVED;
      }
    }
  }
}

ISR (TIMER0_COMPA_vect)
{
  timer_off();
}

ISR(INT0_vect) /* Clock interrupt */
{
  timer_on();
  if (ps2state == PS2_STATE_SENDING)
  {
    bit_send();
  }
  else
  {
    bit_receive();
  }
}

void ps2_send(uint8_t b)
{
  while (ps2state == PS2_STATE_RECEIVING || !(PS2_CLOCK_PIN & _BV(PS2_CLOCK)));

  ps2state = PS2_STATE_SENDING;
  timer_on();

  data.i = 0;
  data.buf[0] = b;

  ps2clock_low();
  _delay_us(110);

  ps2data_low(); /* Start bit */
  _delay_us(5);

  ps2clock_high();

  while (ps2state == PS2_STATE_SENDING);
  timer_off();
}

int main(void)
{
  DDRD = _BV(PD5); /* pin 5 port D */

  serial_setup();
  serial_init();

  timer_init();

  /* Configure PS2_DATA as input with pull-up resistor */
  ps2data_high();

  /* Configure PS2_CLOCK line as input with pull-up resistor. */
  ps2clock_high();
  _delay_ms(100);

  /* Configure INT0 (clock) for falling edge */
  EICRA = _BV(ISC01);
  EIMSK |= _BV(INT0);

  sei();

  _cirbuf_push(&tx, '\r');
  _cirbuf_push(&tx, '\n');
  _cirbuf_push(&tx, 'S');
  UCSR1B |= _BV(UDRIE1);
  
  ps2_send(0xff);
  while (ps2state != PS2_STATE_ACK);
  ps2state = PS2_STATE_IDLE;

  /* BAT */
  while (ps2state != PS2_STATE_RECEIVED);
  ps2state = PS2_STATE_IDLE;

  uint8_t x = 0;

  for (;;)
  {
    x++;
/*    
    ps2_send(0xed);
    _delay_ms(10);

    ps2_send(x & 0x7);
    _delay_ms(10);
*/
    if (ps2state == PS2_STATE_RECEIVED)
    {
      ps2state = PS2_STATE_IDLE;
      _cirbuf_push(&tx, hex(data.buf[0] >> 4));
      _cirbuf_push(&tx, hex(data.buf[0] & 0xf));
      _cirbuf_push(&tx, '\r');
      _cirbuf_push(&tx, '\n');
      UCSR1B |= _BV(UDRIE1);
    }
    
  }
}
