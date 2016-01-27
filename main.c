
// avrdude.exe -c usbtiny -p m328p -U flash:w:avr-dht22.hex U hfuse:w:0xdf:m

// 16MHz clock
#define F_CPU 16000000UL


#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define DHT_LO		PORTC &= ~(1 << DHT_PIN)
#define DHT_HI		PORTC |= (1 << DHT_PIN)

#define DHT_IN		DDRC &= ~(1 << DHT_PIN)
#define DHT_OUT		DDRC |= (1 << DHT_PIN)

#define DHT_READ	(PINC & (1 << DHT_PIN))

#define LED_POWER_ON	PORTB |= (1 << PORTB2)
#define LED_POWER_OFF	PORTB &= ~(1 << PORTB2)
#define LED_SUCCESS_ON	PORTB |= (1 << PORTB1)
#define LED_SUCCESS_OFF	PORTB &= ~(1 << PORTB1)
#define LED_BUSY_ON		PORTB |= (1 << PORTB0)
#define LED_BUSY_OFF	PORTB &= ~(1 << PORTB0)
#define LED_ERROR_ON	PORTD |= (1 << PORTD6)
#define LED_ERROR_OFF	PORTD &= ~(1 << PORTD6)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

volatile unsigned char TMR1_COUNT = 0;
unsigned char TMR1_TARGET = 30; // 30sec

int TMR2_COUNT = 0;
int TMR2_TARGET;
volatile char TMR2_OVERFLOW;

unsigned char DHT_DATA[10]; // 5 bytes per sensor
unsigned char DHT_PIN = 5; // current sensor pin
unsigned char DHT_ERROR; // 1 = error, 0 = ok

volatile char USART_RX_BUFFER[255]; // usart receive buffer
volatile char USART_RX_POS; // current buffer position
volatile char USART_RX_EOL; // did we receive an EOL char?
char USART_RX_STOR = 0; // store received data?

// max = 255 * 4us = 1.020ms
void tmr0_start(int prescale_us){
	TCCR0A |= 0x00; // normal mode
	TCNT0 = 255 - (prescale_us / 4); // preload counter register
	TCCR0B |= (1 << CS00) | (1 << CS01); // set 1:64 prescaler and start
}

void tmr0_stop(){
	TCCR0B = 0;
	TIFR0 &= ~(1 << TOV0); // clear interrupt bit
}


void tmr1_setup()
{
	OCR1A = 0x3D08; // 1sec

	TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
	TIMSK1 |= (1 << OCIE1A); // compare with OCR1A
	TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 and start the timer
}

void tmr2_start() {
	OCR2A = 255;
	TCCR2A |= (1 << WGM21); // set mode 2, CTC
	TIMSK2 |= (1 << OCIE2A);
	TCCR2B = (1 << CS22); // prescaler 1:64 and start timer
}

void tmr2_stop() {
	TCCR2B = 0;
}

void usart_setup(unsigned int ubrr)
{
	// set baud rate register
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	UCSR0B = (1 << RXCIE0) | (1<<RXEN0) | (1<<TXEN0); // enable rx/tx, enable rx interrupt
	UCSR0C = (3<<UCSZ00);
}


void setup(unsigned int ubrr){
	DDRC |= (1 << PORTC0); // C0 output ESP reset pin
	DDRB |= (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2); // B0/B1/B2 output LED
	DDRD |= (1 << PORTD7); // D7 output LED

	usart_setup(ubrr);
	tmr1_setup();

	PORTC |= (1 << PORTC0);

	sei(); // enable interrupts
}

void dht_measure(char offset){
	char dht_overflow;
	
	DHT_OUT;
	DHT_LO;
	_delay_ms(25); // pull low for at least 25ms
	
	DHT_HI;
	DHT_IN;
	tmr0_start(60); // dht must respond within 60us, else timeout
	while(DHT_READ && !TIFR0&1); // pull hi and wait for dht to pull low

	dht_overflow = TIFR0 & 1;
	tmr0_stop();
	
	if(dht_overflow) {
		DHT_ERROR = 1; // dht took longer than 60us to respond
		return;
	}
	
	while(DHT_READ == 0);
	while(DHT_READ); // dht will lo 80us, hi 80us

	// data transmission begins..
	
	unsigned char nbits = 5;
	unsigned char cbit;

	int8_t n, c;

	for(n = 0; n < nbits; n++){
		cbit = 0;
		for(c = 7; c >= 0; --c){
			while(DHT_READ == 0);
			
			_delay_us(40); // delay 40us, then check line state, when hi databit = 1, when low the bit = 0
			if(DHT_READ){
				cbit |= (1 << c);
			}
			
			while(DHT_READ); // wait for new bit
		}

		DHT_DATA[n + offset] = cbit;
	}
}

void usart_tx_byte(unsigned char data)
{
	// wait for tx buffer to be cleared
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void usart_tx_str(char *str){
	while(*str){
		usart_tx_byte(*str);
		str++;
	}
	USART_RX_POS = 0;
}

char usart_wait_for(char *str, int timeout_ms){
	USART_RX_STOR = 1;

	TMR2_COUNT = 0;
	TMR2_OVERFLOW = 0;
	TMR2_TARGET = timeout_ms; // one interrupt = ~980us

	tmr2_start();

	// while no timeout && no usart_eol
	while(!TMR2_OVERFLOW && !USART_RX_EOL) {
		char * match = strstr(USART_RX_BUFFER, str);
		if(match && match - &USART_RX_BUFFER[0] < USART_RX_POS) {
			USART_RX_STOR = 0;
			tmr2_stop();

			for(unsigned char n = 0; n < USART_RX_POS; n++){
				USART_RX_BUFFER[n] = 0;
			}
			
			return 1;
		}

		// wait for another eol
		USART_RX_EOL = 0;
	}

	USART_RX_STOR = 0;
	tmr2_stop();
	return 0;
}

char open_connection(){
	usart_tx_str("AT+CIPSTART=\"TCP\",\"192.168.1.73\",9000\r\n");
	return usart_wait_for((char*)"OK", 1000);
}

char esp_reset(){
	//PORTC &= ~(1 << PORTC0);
	//_delay_ms(100);
	//PORTC |= (1 << PORTC0);
	
	usart_tx_str("AT+RST\r\n");
	_delay_ms(3000);

	usart_tx_str("AT+CWMODE=1\r\n");
	if(usart_wait_for((char*)"OK", 5000)){
		usart_tx_str("AT+CWJAP=\"<SSID>\",\"<PASSWD>\"\r\n");
		if(usart_wait_for((char*)"OK", 10000)){

			usart_tx_str("AT+CIPMUX=0\r\n");
			return usart_wait_for((char*)"OK", 5000);
		}
	}
	
	return 0;
}

char report(){
	uint16_t rhum = (double)((DHT_DATA[0] << 8) + DHT_DATA[1]);
	uint16_t rtmp = (double)((DHT_DATA[2] << 8) + DHT_DATA[3]);

	uint16_t rhum1 = (double)((DHT_DATA[5] << 8) + DHT_DATA[6]);
	uint16_t rtmp1 = (double)((DHT_DATA[7] << 8) + DHT_DATA[8]);

	char len[24];
	char str[128];
	sprintf(str, "POST /update?s1[h]=%d&s1[t]=%d&s2[h]=%d&s2[t]=%d HTTP/1.1\r\nHost: 192.168.1.73\r\n\r\n", rhum, rtmp, rhum1, rtmp1);
	sprintf(len, "AT+CIPSEND=%d\r\n", strlen(str));

	
	usart_tx_str(len);
	if(usart_wait_for((char*)"OK", 5000)){
		usart_tx_str(str);
		return usart_wait_for((char*)"SEND OK", 5000);
	}
	
	return 0;
}


int main(void)
{
	setup(BAUD_PRESCALE);

	LED_POWER_ON;

	char success = 0;
	
	while(1){	
		LED_ERROR_OFF;
		LED_SUCCESS_OFF;
		LED_BUSY_ON;

		if(esp_reset()){
			DHT_ERROR = 0;

			DHT_PIN = 4;
			dht_measure(5);
			DHT_PIN = 5;
			dht_measure(0);

			if(!DHT_ERROR && open_connection()){
				success = report();
			}
		}

		LED_BUSY_OFF;

		if(success){
			LED_SUCCESS_ON;
		} else {
			LED_ERROR_ON;
		}
		
		while(TMR1_COUNT < TMR1_TARGET){
			_delay_ms(10);
		}

		TMR1_COUNT = 0;
		success = 0;
	}
}

// 1sec have passed
ISR (TIMER1_COMPA_vect)
{
	TMR1_COUNT++;
}

// 980us have passed
ISR(TIMER2_COMPA_vect){
	TMR2_COUNT++;
	if(TMR2_COUNT >= TMR2_TARGET){
		tmr2_stop();
		TMR2_OVERFLOW = 1;
	}
}

// when we receive usart byte
ISR(USART_RX_vect){
	char rx_byte = UDR0;
	if(USART_RX_STOR == 1){
		USART_RX_BUFFER[USART_RX_POS++] = UDR0;

		if(rx_byte == "\n"){
			USART_RX_EOL = 1;
		}
	}
}
