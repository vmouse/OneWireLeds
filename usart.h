/*
 * usart.h
 *
 * Created: 07.12.2013 21:48:35
 *  Author: vlad
 */ 


#ifndef USART_H_
#define USART_H_

void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
}

uint8_t USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void USART_transmit(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void USART_putstring(char* StringPtr)
{
	while(*StringPtr != 0x00){  
		USART_transmit(*StringPtr); 
	StringPtr++;}				
}

uint8_t HexToInt(char Hex) {
	uint8_t res = 0;
	if ((Hex>='0') & (Hex<='9')) {
		res = Hex - '0';
		} else if ((Hex>='A') & (Hex<='F')) {
		res = Hex - 'A' + 10;
	}
	return res;
}

void IntToHex(uint8_t value, char* buffer) {
	uint8_t a = value & 0x0f;
	uint8_t b = (value>>4) & 0x0f;
	buffer[1] = (a<10)?'0'+a:'A'+(a-10);
	buffer[0] = (b<10)?'0'+b:'A'+(b-10);
}

void IntToBin(uint8_t value, char* buffer) {
	for (uint8_t i=0; i<8; i++) {
		buffer[i] = ((value & 0x80) == 0)?'0':'1';
		value = value << 1;
	}
}

void USART_puthex(uint8_t data)
{
	char Buf[] = "__";
	IntToHex(data, Buf);
	USART_putstring(Buf);	
}

void USART_putbin(uint8_t data)
{
	char Buf[] = "00000000";
	IntToBin(data, Buf);
	USART_putstring(Buf);
}

#endif /* USART_H_ */