// libraries
#include <QTRSensors.h>
#include <avr/io.h>
#include <avr/interrupt.h<

// set up pins
int buttonPin = 7;
int ledPin = 8;
volatile int buttonState = 0;

// interrupts
float t = 0;
//const unsigned int LED_PIN = 13;
//volatile unsigned int led = 0;



int leftPower = 5;
int leftDirection = 6;
int rightPower = 10;
int rightDirection = 11;
int sensor1 = 2;
int sensor2 = 3;
int graySensor = 4;
int speaker = 9;

byte LED_PIN = 10;
byte LED_BIT = 2;

boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;

int t = 0;

frequency = analogRead(A4);


void setup() 
{ 
  // set up button press  
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(0, pin_ISR, CHANGE);

  // set up interrupt
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led);

  FTM0_SC = 0;
  FTM0_CNT = 0;
  FTM0_MOD = 0xFFF;
  FTM0_SC = 0b011001111;
  NVIC_SET_PRIORITY(IRQ_FTM0, 64);
  NVIC_ENABLE_IRQ(IRQ_FTM0);
  
  // set up other outputs
  pinMode(leftPower, OUTPUT);  
  pinMode(rightPower, OUTPUT);
  pinMode(leftDirection, OUTPUT);  
  pinMode(rightDirection, OUTPUT); 
  pinMode(sensor1, OUTPUT);
  pinMode(sensor2, OUTPUT);
  pinMode(graySensor, OUTPUT);
  pinMode(speaker, OUTPUT);

  Serial.begin(9600);
  
  delay (2000);

// setup interrupt
  DDRD = 0xFF;
  DDRB = 0xFF;
  cli(); // stop interrupt

  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 49;
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts

}//end setup

  
ISR(TIMER2_COMPA_vect)
{
  t+=1;
  if (t==628)
    t=0;
}



// main loop
void loop() 
{    
  NVIC_DISABLE_IRQ(IRQ_FTM0);
  delay(50);
  NVIC_ENABLE_IRQ(IRQ_FTM0);
  
  int power = 0;
  sensorA = digitalRead(sensorValue[1]);
  sensorB = digitalRead(sensorValue[2]);

  while (sensorA ==0 && sensorB ==0)
  {  
    drive(50);
    delay(1200000);
    grayscale();
    
    while (sensorA !=0 || sensorB !=0)
      lineFollow();
  }

  
 }

// button press

void pin_ISR()
{
  buttonState = digitalRead(buttonPin);
  digitalWrite(ledPin, buttonState);
}

// line follow loop
void lineFollow(int sensorA)
{  
    if (sensorA == HIGH)
    {
      digitalWrite(leftPower,HIGH);
      digitalWrite(leftDirection,LOW);
      digitalWrite(rightPower,LOW);
      digitalWrite(rightDirection,LOW);
     }
    else
    {
      digitalWrite(rightPower,LOW);
      digitalWrite(rightDirection,HIGH);
      digitalWrite(leftPower,LOW);
      digitalWrite(leftDirection,LOW);
    }
  }


// drive at power function
void drive(int power)
{
  motor[motorA] = motor[motorC] = power;
}


// detect grayscale
void grayscale()
{
  int grayScale = 0;
  grayScale = analogRead(0);
  Serial.println(grayScale,DEC);
  delay (100);
}


// play music
int speakerOut (int tempo, int elapsedTime, int duration, int pause)
{
  while (elapsedTime < duration)
  {
    digitalWrite(speakerOut, HIGH);
    delay (2000);
    elapsedTime += elapsedTime;
  }
}

int sineWave (int time)
{
  time = time();
  PORTD = byte(127+127*sin(time/100));
}

double time()
{
  int size = 256;
  int array[size];

  for (count = 1; count < size; count++)
  {
    count +=0.1;
    printf()
    return count;  
  }
}


