#include <avr/io.h>
#include <avr/interrupt.h>

int x, y;
int setpoint=35;
float motorSpeed = 0.0;
float Kp = 0.1, Kd = 0.0;
float pwm1 = 50.0, pwm2 = 50.0, pwm3 = 50.0;

#define m1pwm 10
#define m1dir 9
#define m2pwm 12
#define m2dir 11
#define m3pwm 7
#define m3dir 8

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
  digitalWrite(m3dir, HIGH);
  analogWrite(m1pwm, 0);
  
 }
 
void loop() {

  if (Serial2.available() > 0 )
  {
    x = Serial2.read();

    int error = x - setpoint;   // Calculate the deviation from position to the set point
    motorSpeed = Kp * error;
    pwm3 = pwm3 - motorSpeed;
    pwm2 = pwm2 + motorSpeed;

    digitalWrite(m2dir, LOW);
    digitalWrite(m3dir, HIGH);
    pwm2=constrain(pwm2, 10, 100);
    pwm3=constrain(pwm3, 10, 100);
    analogWrite(m2pwm, pwm2);
    analogWrite(m3pwm, pwm3);
    Serial.print("   x= ");
    Serial.print(x);
    Serial.print("   pwm2= ");
    Serial.print(pwm2);
    Serial.print("   pwm3= ");
    Serial.print(pwm3);
    Serial.println();
    


  }
}

