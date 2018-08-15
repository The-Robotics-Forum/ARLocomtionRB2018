#include <HMC5883L.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <Wire.h>

HMC5883L compass;

#define m1dir 10    //motor 1 direction pin
#define m2dir 12    
#define m4dir 7
#define m3dir 9
#define m1pwm 11    //motor 1 pwm pin
#define m2pwm 5
#define m4pwm 6
#define m3pwm 8

 volatile int count=0;   //pluse count of first encoder
 volatile int count2=0;  //pluse count of second encoder
 int pwm1=10, pwm2=10;
 float headingDegrees, setPoint;
 float declinationAngle, heading;
 
void hmc_initialize()
{
   Serial.println("Initialize HMC5883L");   // Initialize HMC5883L
 while (!compass.begin())
 {
   Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
   delay(500);
 }
  compass.setRange(HMC5883L_RANGE_1_3GA);   // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);   // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_30HZ);   // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8);   // Set number of samples averaged
  compass.setOffset(0, 0);   // Set calibration offset. See HMC5883L_calibration.ino
  
}

int hmcCall()
{
  Vector norm = compass.readNormalize();
 float heading = atan2(norm.YAxis, norm.XAxis);// Calculate heading

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  float headingDegrees = heading * 180/M_PI;    // Convert to degrees

  return (headingDegrees);
}

void hmc_check()
{
  if(headingDegrees<setPoint-0.5 && pwm2>0)      //bot is moving towards left
{
  pwm2=pwm2-5;
  analogWrite(m2pwm,pwm2);
  analogWrite(m1pwm,pwm1);
}

else if(headingDegrees>=setPoint-0.5 && headingDegrees<setPoint+0.5)    //bot is on the white line-->no change in pwm //1 degrees tolerance is set
{
  analogWrite(m2pwm,pwm2);
  analogWrite(m1pwm,pwm1);
}

else if (headingDegrees>=setPoint+0.5 && pwm1>0)    //bot is moving towards right
{
  pwm1=pwm1-2;
  analogWrite(m2pwm,pwm2);
  analogWrite(m1pwm,pwm1);
  
}

}

void setup() {
 Serial.begin(9600);
 
 //setPoint=headingDegrees;

 hmc_initialize();
pinMode(m2dir,OUTPUT);
pinMode(m1dir,OUTPUT);
pinMode(m4dir,OUTPUT);
pinMode(m3dir,OUTPUT);
pinMode(m2pwm,OUTPUT);
pinMode(m1pwm,OUTPUT); 
pinMode(m4pwm,OUTPUT);
pinMode(m3pwm,OUTPUT);
setPoint= hmcCall();
 DDRD=0b00000000;     //setting pin0 and pin1 as input of portD
 DDRE=0b00000000;     //setting pin4 and pin5 as input of portE
  
 PORTD|=(1<<2)|(1<<3); //pull up the input
 PORTE|=(1<<5)|(1<<4);  //pull up the input
 EIMSK|= (1<<INT2)|(1<<INT3)|(1<<INT4)|(1<<INT5); //enabling external interrupt 
 EICRA=0b11110000; //setting rising edge for INT2 and INT3 
 EICRB=0b00001111; //setting rising edge for INT4 and INT5 
 sei(); //global interrupt

}


void loop()
{  

  
headingDegrees=hmcCall();  

//for the first white path, motor 3 and 4 will work:
// following commented section only accelerates and decelerates the bot for first desired path.
// It does take the readings from HMC to check the deflection.
   
/*digitalWrite(m2dir,LOW);
  digitalWrite(m1dir,LOW);
  digitalWrite(m4dir,HIGH);

  digitalWrite(m3dir,LOW);
  analogWrite(m2pwm,0);
  analogWrite(m1pwm,0);

if(count<128)
{
  pwm=pwm+5;
  analogWrite(m4pwm,pwm);
  analogWrite(m3pwm,pwm);

}

if(count>128 && count<256)
{
  analogWrite(m4pwm,pwm);
  analogWrite(m3pwm,pwm);
}

if(count>256 && count <1200 && pwm>0)
{
  pwm=pwm-5;
  analogWrite(m4pwm,pwm);
  analogWrite(m3pwm,pwm);

}

if(count>=1200 && count<1300)
{
  pwm=0;
  analogWrite(m4pwm,pwm);
  analogWrite(m3pwm,pwm);
}
if(count>1300)
{
  digitalWrite(m4dir,LOW);
  digitalWrite(m3dir,HIGH);
   analogWrite(m4pwm,25);
  analogWrite(m3pwm,25);
  
}*/

// to turn right for second white path, motors 1 and 2 will work:
// it will also interface with HMC :

  digitalWrite(m2dir,HIGH);
  digitalWrite(m1dir,LOW);
  digitalWrite(m4dir,LOW);
  digitalWrite(m3dir,LOW);
  analogWrite(m4pwm,0);
  analogWrite(m3pwm,0);


if(count2<=1654)            // bot will accelerate slowly 
{ 
  pwm1=pwm1+5;
  pwm2=pwm2+5;
  analogWrite(m1pwm,pwm1);
  analogWrite(m2pwm,pwm2);
  hmc_check();             // check if bot is going out of the white line
  if(pwm1>=100 && pwm2>=100)  // maximum limit of pwm is set o 100 
  {
    pwm1=100;
    pwm2=100;
  }
  
}

if(count2>1654 && count2<=3721)  // bot will move at constant speed
{
  analogWrite(m1pwm,pwm1);
  analogWrite(m2pwm,pwm2);
  hmc_check();
  if(pwm1>=100 && pwm2>=100)
  {
    pwm1=100;
    pwm2=100;
  }
}

if(count2>3721&& count2<=8000 && pwm1>0 && pwm2>0)   // speed will decelerate 
{
  pwm1=pwm1-5;
  pwm2=pwm2-5;
  analogWrite(m1pwm,pwm1);
  analogWrite(m2pwm,pwm2);
  hmc_check();
}

if(count2>8000 && count2<=8270) 
{
  pwm1=0;
  pwm2=0;
  analogWrite(m1pwm,pwm1);
  analogWrite(m2pwm,pwm2);
  hmc_check();
}
if(count2>8270)      // if the bot crosses desired distance then it will come back to the desired position
{
  digitalWrite(m2dir,LOW);
  digitalWrite(m1dir,HIGH);
   analogWrite(m1pwm,25);
  analogWrite(m2pwm,25);
  hmc_check();
  
}

  // Output
 Serial.print("headingDegrees=");
 Serial.print(headingDegrees);
 Serial.print("  count2=");
 Serial.print(count2);
 Serial.print("  setPoint=");
 Serial.print(setPoint);
 Serial.println();
 delay(100);
}

  
  


//first interrupt
ISR(INT2_vect)
{
  if((PIND &(1<<3))) //at rising edge if b=0, it is moving in clockwise direction 
    {
      count--; 
        
    }  
}

//2nd interrupt
ISR(INT3_vect)
{
 
  if((PIND &(1<<2)))
    {
      count++;
        
    }
}

//3rd interrupt
ISR(INT4_vect)
{
  if((PINE &(1<<5))) //at rising edge if b=0, it is moving in clockwise direction 
    {
      count2++; 
        
    }  
}

//4th interrupt
ISR(INT5_vect)
{
 
  if((PINE &(1<<4)))
    {
      count2--;
        
    }
}
