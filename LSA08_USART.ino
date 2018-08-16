#include <avr/io.h>
#include <avr/interrupt.h>


int x, y;
int setpoint = 35;
float motorSpeed = 0;
float Kp = 0.2, Kd = 0;
float pwm1 = 50, pwm2 = 50, pwm3 = 50;

#define m1pwm 10
#define m1dir 9
#define m2pwm 12
#define m2dir 11
#define m3pwm 7
#define m3dir 8

void USART_Init3(unsigned int baud)
{
  // set baud rate
  UBRR3H = (unsigned char)(baud >> 8);
  UBRR3L = (unsigned char) baud;
  // enable reciever and transmitter
  UCSR3B = (1 << TXEN3) | (1 << RXEN3);
  // set frame format: 8data, 2 stop bit
  UCSR3C = (1 << USBS3) | (3 << UCSZ30);

}
void USART_Init2(unsigned int baud)
{
  // set baud rate
  UBRR2H = (unsigned char)(baud >> 8);
  UBRR2L = (unsigned char) baud;
  // enable reciever and transmitter
  UCSR2B = (1 << TXEN2) | (1 << RXEN2);
  // set frame format: 8data, 2 stop bit
  UCSR2C = (1 << USBS2) | (3 << UCSZ20);

}

unsigned char USART_RECIEVE2(void)
{
  //wait for data to be recieved
  while (!(UCSR2A & (1 << RXC2)))
    ;
  //get and recieve data from buffer
  return UDR2;
}
unsigned char USART_RECIEVE3(void)
{

  //wait for data to be recieved
  while (!(UCSR3A & (1 << RXC3)));
  //get and recieve data from buffer
  return UDR3;

}
void setup() {
  Serial.begin(9600);
  //Serial3.begin(9600);
  //Serial2.begin(9600);
  pinMode(m1pwm, OUTPUT);
  pinMode(m2pwm, OUTPUT);
  pinMode(m3pwm, OUTPUT);
  pinMode(m1dir, OUTPUT);
  pinMode(m2dir, OUTPUT);
  pinMode(m3dir, OUTPUT);

  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, LOW);
  digitalWrite(m3dir, LOW);
  analogWrite(m1pwm, 0);
  analogWrite(m2pwm, 0);
  analogWrite(m3pwm, 0);
}
void loop() {

  x = USART_RECIEVE2();
  int error = x - setpoint;   // Calculate the deviation from position to the set point
  motorSpeed = Kp * error;
  pwm3 = pwm3 - motorSpeed;
  pwm2 = pwm2 + motorSpeed;
   Serial.print("   x= ");
  Serial.print(x);
  Serial.print("   pwm2= ");
  Serial.print(pwm2);
  Serial.print("   pwm3= ");
  Serial.print(pwm3);
   pwm2=constrain(pwm2, 10, 100);
   pwm3=constrain(pwm3, 10, 100);
  digitalWrite(m2dir, LOW);
  digitalWrite(m3dir, HIGH);
  analogWrite(m2pwm,pwm2 );
  analogWrite(m3pwm,pwm3);

  Serial.print("   x2= ");
  Serial.print(x);
  Serial.print("   pwm22= ");
  Serial.print(pwm2);
  Serial.print("   pwm32= ");
  Serial.print(pwm3);
  Serial.println();

}

