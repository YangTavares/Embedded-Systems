#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#include <avr/io.h>
#include <avr/interrupt.h>
/* WG3-0 = 0100 = CTC TOP OCR3A TOV3 MAX
 * Timer interrupt for 3 min project
 * TCCR3A = 11 set on compare 0000 unused 00 WGM1-0 
 * TCCR3B = 00 unused 0 R 01 WGM3-2 011 prescale = 64
 * TIMSK3 = 00 R 0 enable interrupt 0 R 000 no ports used 1 en overflow interrupt
 */

 
/*
 * PB7 LED 13 BUILTIN
 * DDRB sets directions of pins PB7-0 0=input
 * TCCR0A configure PWM for 0C0A - PB7
 * 
*/

/*
 * PB6-5 PWM PORT 12-11
 * DDRB
 * COMnA1 = 11 Set OCnA on compare match
 * TCCR1A = 11 11 11 01 // 01 = WGMn1
 * TCCR1B = 00 noise 0 reserved 01 WGM132  011 64 clk
 * WGMn3-0 0101 Fast PWM 8-bit TOP 0x00FF
 */

 /*
  * PB4 PWM PORT 10
  *  TCCR2A = 11 11 Set on compare match 00 reserved 11 Fast PWM
  *  TCCR2B = 00 no force output 00 reserved 0 fastp PWM 100 64 clk
  */
 /*
  * ADMUX = 01 AVCC 5V 0 revert result? 00000 ADC0
  * ADCSRA = 1 enable 0 start 0 auto trigger 0 interrupt 1 enable interrupt 111 clk 128
  * ADCSRB &= 11110111; MUX5 0
  */

void read_adc();

 
//Declaration of our functions
void USART_init(void);
unsigned char USART_receive(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);

volatile int test=0;
volatile int count=0;
volatile float minute=0.0;
volatile float f_secagem=0.0;
volatile bool flag=0;
volatile uint8_t light=0;
volatile uint8_t temperature=0;
  
volatile float f_fan=0.0;
volatile float gama = 45262.0;
volatile float beta = 1;

ISR(TIMER3_COMPA_vect){
   //0,25 sec
  //if(count%4==0) //aprox 1 se
  if(flag==1){
    count++;
    ADMUX &= 0b11111110; //A0
    read_adc();
    light = ADC >> 2;
    //OCR0A = light;
    ADMUX |= 0b00000001;//A1
    read_adc();
    temperature = ADC >> 2;
    OCR1B = temperature;
    minute = count/240.0;
    
    if(count < 120){ // < 0,5 min
        f_secagem = 0.9*minute;
      }
      else if(count >=120 && count < 360){ // < 1,5 min
        f_secagem = 0.45;
      }
      else if(count >= 360 && count < 480){ // < 2 min
        f_secagem = 0.7*minute - 0.6;
      }
      else if(count >=480 && count < 648){ // < 2,7 min
        f_secagem = 0.8;
      }
      else if(count >=648 && count <= 720){ // < 3 min
        f_secagem = -2.6667*minute + 8.0;
      }
      else{
        flag=0;
      }
      
      f_fan = (((f_secagem*(gama/temperature)) -1*beta*light)+255.0)/2.0;
      if (f_fan > 255){
        f_fan = 255;
      }
    
      if (f_fan < 0){
        f_fan = 0;
      }
      OCR0A = 255 -((int)(f_fan));
      
      char int_str[1000];
      sprintf(int_str, "%d", light);
      USART_putstring(int_str);
      USART_putstring(" ");
      sprintf(int_str, "%d", temperature);
      USART_putstring(int_str);
      USART_putstring(" ");
      sprintf(int_str, "%d", (int)f_fan);
      USART_putstring(int_str);
      USART_putstring(" ");
      sprintf(int_str, "%d", (int)(minute*60.0));
      USART_putstring(int_str);
      USART_putstring("\n");

      OCR1A = 255-temperature;
      OCR1B = light;
      //PINB |= 0b00010000;
    }
     else{
        count=0;
        EIMSK |= 1<<INT0;
        OCR1A = 0x00FF;
        OCR1B = 0x00FF;
        PINB &= ~0b00010000;
        OCR0A = 255;
      }
}

ISR(INT0_vect){
  flag=1;
  EIMSK &= 0b11111110;
  PINB |= 0b00010000;
  
}
int main(){
  USART_init();  
  /*
   * config time interrupt
   */
  TCCR3A = 0b11000000; 
  TCCR3B = 0b00001011;
  TIMSK3 = 0b00000011;

  //Counter Top for overflow
  // 0,25/64 = x; x/0,000000063 = 62004 aprox; 1/16M = 6,3e-8
  OCR3A  = 0xF234;
 
  // Enable INT0 External Interrupt port 21
  EIMSK |= 1<<INT0;
  //Enable PD0 as input
  DDRD &= 0;
  //Activate interrupts
  sei();
  /*
   * ADC config
   */
  ADMUX = 0b01000000;
  ADCSRA |= 0b10000111;
  //ADCSRA &= 0b01111111;
  //ADCSRB &= 11110111;
  
  DDRB|=0b11110000;//need?Yes TCCR0A last 2 bits overrides port func
  PORTB = 0b00000000;
  
  DDRH|=0b01000000; //PORTH6 pin9
  /*
   * Fast PWM mode
   * Set OC0A on Compare Match, clear OC0A at BOTTOM
   * (inverting mode)
   */
  TCCR0A = 0b11000011;
  
  TCCR1A = 0b11111101;

  //TCCR2A = 0b11110011;
  /*
   * clk divided by 64
   * Top = 0xFF
   * need fixed top to change duty cycle
   */
  TCCR0B = 0b00000011;
  
  TCCR1B = 0b00001011;

  //TCCR2B = 0b00000100;
  
  OCR1A = 0x00FF; //PIN 11 PB5
  OCR1B = 0x00FF; //PIN 12 PB6 16-bit
  OCR0A = 0xFF; //PIN 13 duty cycle
  OCR2A = 0xFF; //PIN 10
  OCR2B = 0xFF;//PIN 9


ADMUX |= 0b00000001;
//count=640;
  while (1){
    //secagem test
    //A0 = LIGHT
    //A1 = TEMPERATURE
    //L0 P LIGHT = 11 --> OCR1A
    //L1 P TEMPERATURE = 12 -->OCR1B
    //OCR1B = temperature;
    //OCR1A = light;
  }
}

void read_adc(){
  ADCSRA|=(1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  ADCSRA |= (1<<ADIF);
  }
void USART_init(void){
 
 UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
 UBRR0L = (uint8_t)(BAUD_PRESCALLER);
 UCSR0B = (1<<RXEN0)|(1<<TXEN0);
 UCSR0C = (3<<UCSZ00);
}
 
void USART_send( unsigned char data){
 
 while(!(UCSR0A & (1<<UDRE0)));
 UDR0 = data;
 
}
 
void USART_putstring(char* StringPtr){
 
while(*StringPtr != 0x00){
 USART_send(*StringPtr);
 StringPtr++;}
 
}

