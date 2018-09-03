#include <avr/io.h>
#include <avr/interrupt.h>

volatile int x, y;
int setpoint = 35, error = 0, error2 = 0;
float motorSpeed = 0, motorSpeed2 = 0, Kp = 0.6;
float pwm1 = 0, pwm2 = 0, pwm3 = 0;
bool flag1 = 1, flag2 = 1;

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
void x_recieve(int x)
{
  pwm1=0;
  // Calculate the deviation from position to the set point
  error = x - setpoint;   
  if (error > 15)
  {
    error = 15;
  }
  if (error < -15)
  {
    error = -15;
  }
  motorSpeed = Kp * error;
  pwm3 = 60 - motorSpeed;
  pwm2 = 60 + motorSpeed;
  pwm2 = constrain(pwm2, 20, 80);
  pwm3 = constrain(pwm3, 20, 80);
   if (x > 35 && x <= 70)
  {
    pwm1 = 15 + motorSpeed;
    digitalWrite(m1dir, HIGH);
  }
  else  if (x < 35)
  {
    pwm1 = 15 + motorSpeed;
    digitalWrite(m1dir, LOW);
  }
  else  if (x == 35)
  {
    pwm1 = 0;
    digitalWrite(m1dir, HIGH);
  }
  flag1 = 0;
  flag2 = 1;
  digitalWrite(m2dir, LOW);
  digitalWrite(m3dir, HIGH);
  analogWrite(m1pwm, pwm1);
  analogWrite(m2pwm, pwm2 );
  analogWrite(m3pwm, pwm3);

  Serial.print("   x= ");
  Serial.print(x);
  Serial.print("   y= ");
  Serial.print(y);
  Serial.print("   pwm2= ");
  Serial.print(pwm2);
  Serial.print("   pwm3= ");
  Serial.print(pwm3);
  Serial.print("   pwm1= ");
  Serial.print(pwm1);
  Serial.print("  error");
  Serial.print(error);
  Serial.println();
}
void y_recieve(int y)
{
  pwm3 = 0;
  analogWrite(m3pwm, pwm3);
  // Calculate the deviation from position to the set point
  error2 = y - setpoint;   
  if (error2 > 15)
  {
    error2 = 15;
  }
  if (error2 < -15)
  {
    error2 = -15;
  }
  motorSpeed2 = Kp * error2;
  pwm1 = 60 + motorSpeed2;
  pwm2 = 60 - motorSpeed2;
  pwm2 = constrain(pwm2, 20, 80);
  pwm1 = constrain(pwm1, 20, 80);

  digitalWrite(m2dir, HIGH);
  digitalWrite(m1dir, LOW);
  analogWrite(m1pwm, pwm1);
  analogWrite(m2pwm, pwm2 );
  flag2 = 0;
  flag1 = 1;
  Serial.print("   x= ");
  Serial.print(x);
  Serial.print("   y= ");
  Serial.print(y);
  Serial.print("   y_pwm2= ");
  Serial.print(pwm2);
  Serial.print("   y_pwm1= ");
  Serial.print(pwm1);
  Serial.print("   y_pwm3= ");
  Serial.print(pwm3);
  Serial.print("  y_error2");
  Serial.print(error2);
  Serial.println();
}
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial2.begin(9600);
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
  y = USART_RECIEVE3();
  if (x >= 0 && x <= 70 && y == 255)
  {x_recieve(x);}
  if (y >= 0 && y <= 70 && x == 255)
  {y_recieve(y);}
  if (x == 255 && y == 255 && flag1 == 0)
  { pwm1 = 30;
    pwm2 = 30;
    pwm3 = 0;
    digitalWrite(m1dir, HIGH);
    digitalWrite(m2dir, LOW);
    analogWrite(m1pwm, pwm1);
    analogWrite(m2pwm, pwm2);
    analogWrite(m3pwm, pwm3);
  }
  else if (x == 255 && y == 255 && flag2 == 0)
  {
    pwm1 = 0;
    pwm3 = 30;
    pwm2 = 30;
    digitalWrite(m3dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m3pwm, pwm3);
    analogWrite(m2pwm, pwm2);
    analogWrite(m1pwm, pwm1);
  }
}


