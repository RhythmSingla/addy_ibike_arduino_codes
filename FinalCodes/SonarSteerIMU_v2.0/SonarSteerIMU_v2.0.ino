/*
 * AVR_Sonar.c
 *
 * Created: 4/2/2015 7:23:19 PM
 *  Author: Adarsh Kosta
 */ 


/*
 * Sonar_Interfacing_Atmega16.c
 *
 * Created: 21-Oct-14 9:27:06 PM
 *  Author: Adarsh Kosta
 */ 


#define F_CPU 16000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */


#define TICKS_PER_MS 15984               // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 40      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_US 200 // echo cancelling time between sampling
#define THRESHOLD 130

//#define THRESHOLD 15
#define LEAST_COUNT 6

#define ROT_ANGLE 180
#define PULSE_COUNT 5*ROT_ANGLE

#define PERIOD 60000

#define OPT 8
#define OPT1 2
#define ANGLE_LIMIT 8
#define LIMIT1 2
//#define LIMIT2 5

#define DIST_LIMIT 500
#define MAX_ADC 500

#define LEDPIN 13
#define PULPIN 11
#define DIRPIN 8
#define STOPPIN 23

#define ECHO1 2
#define TRIG1 6
#define ECHO2 3
#define TRIG2 7
#define LEDPIN 13

#define NUM_OF_READINGS 5

volatile long result1 = 0;
volatile unsigned char up1 = 0;
volatile unsigned char running1 = 0;
volatile uint32_t timerCounter1 = 0;

volatile long result2 = 0;
volatile unsigned char up2 = 0;
volatile unsigned char running2 = 0;
volatile uint32_t timerCounter2 = 0;

volatile  uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS*TICKS_PER_MS; 
//  = 480000 this could be replaced with a value instead of multiplying

volatile int distCnt[2] = {0};
volatile int distData1[NUM_OF_READINGS] = {0}; 
volatile int distData2[NUM_OF_READINGS] = {0};
volatile int distSum1 = 0, distSum2 = 0;

volatile int dist[2] ={1000, 1000};

volatile int currentAngle = 0;
volatile int count = 0;
volatile int flag = 0;
volatile int Pulse_Count = 0; 
volatile int value = 0;

volatile int obstacleFlag = 0, returnFlag = 0;
volatile int cycle = 0;
volatile int k = 0;
volatile int time = 0, prev_time = 0, exec_time = 0;

void setPin(int pin)
{
	digitalWrite(pin, HIGH);
}

void resetPin(int pin)
{
	digitalWrite(pin, LOW);
}

double getMax(double a, double b)
{
	if(abs(a) > abs(b))
	return a;
	else
	return b;
}

void setOCR(int value)
{
   OCR1A = value; 
}
void setICR(int value)
{
   ICR1 = value; 
}

void timInitPWM()
{ 
  pinMode(PULPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
   
  setOCR(0);     // compare match register 16MHz/256/2Hz           /// Motor Pulse given @ 10kHz
  setICR(PERIOD);
  
  TCCR1A |= (1<<COM1A1)|(1<<WGM11); //Fast PWM Mode with ICR1 as TOP, Clear on Compare Match
  TCCR1B |= (1<<WGM12)|(1<<WGM13);  
  TCCR1B |= (1 << CS10); // (1<<CS11);// for 64 prescaler    CURRENTLY     // 8 prescaler 
  //TIMSK1 |= (1 << OCIE1A);          // enable timer compare interrupt
  TIMSK1 |= (1<<TOIE1);
}

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  //digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
  
  if(count < Pulse_Count && flag == 1)
  {  
    count++;
  }
  else
  {
    flag = 0;
    setOCR(0);
    count = 0;
    Pulse_Count = 0;
  }  
}

void changeAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*OPT;
     setOCR(PERIOD/5);
     while(flag);
     //delayMicroseconds(100);  
}

void changeRAngle(int dir)
{
     if(dir == -1)
     setPin(DIRPIN);
     else if(dir == 1)
     resetPin(DIRPIN);
     else
     return;
     
     flag = 1;
     Pulse_Count = 5*OPT1;
     setOCR(PERIOD/5);
     while(flag);
     //delayMicroseconds(100);  
}

// timer overflow interrupt, each time when timer value passes 255 value
ISR(TIMER3_OVF_vect)
{      
    if (up1) 
    {       // voltage rise was detected previously
    timerCounter1++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks1 = timerCounter1 * 65535 + TCNT3;
        if (ticks1 > max_ticks) 
      {
          // timeout
          up1 = 0;          // stop counting timer values
          running1 = 0; // ultrasound scan done
          result1 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}

// timer overflow interrupt, each time when timer value passes 255 value
ISR(TIMER4_OVF_vect)
{      
    if (up2) 
    {       // voltage rise was detected previously
    timerCounter2++; // count the number of overflows
    // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
    uint32_t ticks2 = timerCounter2 * 65535 + TCNT4;
        if (ticks2 > max_ticks) 
      {
          // timeout
          up2 = 0;          // stop counting timer values
          running2 = 0; // ultrasound scan done
          result2 = 1000; // show that measurement failed with a timeout (could return max distance here if needed)
      }
    }    
}


ISR(INT4_vect)
{
  if (running1) 
  { //accept interrupts only when sonar was started
    if (up1 == 0) 
    { // voltage rise, start time measurement
        up1 = 1;
        timerCounter1 = 0;
        TCNT3 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up1 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result1 = (timerCounter1 * 65535 + TCNT3)/940;
        running1 = 0;
     }
  }
}

ISR(INT5_vect)
{
  if (running2) 
  { //accept interrupts only when sonar was started
    if (up2 == 0) 
    { // voltage rise, start time measurement
        up2 = 1;
        timerCounter2 = 0;
        TCNT4 = 0; // reset timer counter
    }
    else 
    {        // voltage drop, stop time measurement
        up2 = 0;
        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
        result2 = (timerCounter2 * 65535 + TCNT4)/940;
        running2 = 0;
     }
  }
}

void tim3_Init()
{
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCCR3B |= (1<<CS30); // select internal clock with no prescaling
  TCNT3 = 0; // reset counter to zero
  TIMSK3 = 1<<TOIE3; // enable timer interrupt 
}

void tim4_Init()
{
  TCCR4A = 0;
  TCCR4B = 0;
  
  TCCR4B |= (1<<CS40); // select internal clock with no prescaling
  TCNT4 = 0; // reset counter to zero
  TIMSK4 = 1<<TOIE4; // enable timer interrupt 
}

void enableSonar(int num)
{
  if(num == 1)
 {
   // turn on interrupts for INT4, connect Echo to INT4
  EIMSK |= (1 << INT4); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC40);      // Turns on INT4
 } 
 if(num == 2)
 {
  // turn on interrupts for INT5, connect Echo to INT5
  EIMSK |= (1 << INT5); // enable interrupt on any(rising/droping) edge
  EICRB |= (1 << ISC50);      // Turns on INT5
 }
}

void sonar(int num) 
{
  if(num == 1)
  {
  resetPin(TRIG1);
  delayMicroseconds(1);
  setPin(TRIG1);
  delayMicroseconds(10);
  resetPin(TRIG1);
  delayMicroseconds(1);
  running1 = 1;	
  }
  else if(num == 2)
  {
  resetPin(TRIG2);
  delayMicroseconds(1);
  setPin(TRIG2);
  delayMicroseconds(10);
  resetPin(TRIG2);
  delayMicroseconds(1);
  running2 = 1;	
  }		    
}

void handleObstacle(int cnt)
{  
    while(cnt--)
    {
      if(running1 == 0)
      {
        distData1[distCnt[0]] = result1;
        distCnt[0]++;
        
        if(distCnt[0] == NUM_OF_READINGS)
          distCnt[0] = 0;
        int n;  
        distSum1 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum1 += distData1[n]; 
        }  
                
        dist[0] = distSum1/NUM_OF_READINGS;
        
        if(dist[0] > DIST_LIMIT && dist[0] != 1000)
          dist[0] = DIST_LIMIT;
        
        sonar(1); // launch measurement 
       // delayMicroseconds(DELAY_BETWEEN_TESTS_US);
      }
      
      if(running2 == 0)
      {
        distData2[distCnt[1]] = result2;
        distCnt[1]++;
        
        if(distCnt[1] == NUM_OF_READINGS)
          distCnt[1] = 0;
        int n;  
        distSum2 = 0;
        for(n=0; n<NUM_OF_READINGS; n++)
        {
          distSum2 += distData2[n]; 
        }  
                
        dist[1] = distSum2/NUM_OF_READINGS;
        
        if(dist[1] > DIST_LIMIT  && dist[1] != 1000)
          dist[1] = DIST_LIMIT;
        
         sonar(2); 
       // delayMicroseconds(DELAY_BETWEEN_TESTS_US);
      }
    }
}

void turn()
{
 
 if(dist[0] < THRESHOLD && dist[1] < THRESHOLD)
 {
   returnFlag = 0;
   digitalWrite(STOPPIN, LOW);
   Serial.print(" Stopping\t");
   /*
   if(dist[0] < dist[1] && currentAngle > -ANGLE_LIMIT)
   {
     obstacleFlag = 1;
     Serial.print("\tRIGHT");
     changeAngle(1);
     cycle = 0;
   }
   else if(dist[0] > dist[1] && currentAngle < ANGLE_LIMIT)
   {
     obstacleFlag = 2;
     Serial.print("\tLEFT");
     changeAngle(-1);
     cycle = 0;
   }
   */
 }  
 else if(dist[0] < THRESHOLD && dist[1] > THRESHOLD && abs(currentAngle) < ANGLE_LIMIT)
 {
   //setPin(LEDPIN);
   digitalWrite(STOPPIN, HIGH);   
   returnFlag = 0;
   obstacleFlag = 1;
   Serial.print(" Going Right\t");
   changeAngle(1);
   cycle = 0;
 }
 else if(dist[0] > THRESHOLD && dist[1] < THRESHOLD && abs(currentAngle) < ANGLE_LIMIT)
 {
   //setPin(LEDPIN);
   digitalWrite(STOPPIN, HIGH); 
   returnFlag = 0;
   obstacleFlag = 2;
   Serial.print(" Going Left\t");
   changeAngle(-1);
   cycle = 0;
 }
 else if(dist[0] > THRESHOLD && dist[1] > THRESHOLD)
 {
   digitalWrite(STOPPIN, HIGH);
   
   if(abs(currentAngle) > LEAST_COUNT && returnFlag == 0)
   {
       Serial.print("\tRETURNING");
      if(currentAngle < 0)
      {
        changeAngle(1);
        delay(10);
        k = -1;
      }
      else
      {
        changeAngle(-1);
        delay(10);
        k = 1;
      }
   }
  else if(returnFlag == 0)
  { 
    returnFlag = obstacleFlag;
    obstacleFlag = 0;
    k = 0;
  }
 }
}

void planReturn()
{ 
  if(obstacleFlag == 0 && returnFlag != 0)       // Turned Right, Turn Left more then turn right less for repositioning
  {   
     // Returning Back
     if(returnFlag == 1 && abs(currentAngle) < ANGLE_LIMIT-LIMIT1 && cycle == 0)
       { 
         changeAngle(-1);
         //Serial.println("HERE");
       }
     else if(returnFlag == 2 && abs(currentAngle) < ANGLE_LIMIT-LIMIT1 && cycle == 0)
       {
         changeAngle(1);
       }
     else if(cycle == 0)
       {
         cycle = 1; 
         //delay(500);   /// Delay for optimal turning
       }
     
     if(cycle == 1 && abs(currentAngle) > LEAST_COUNT)
     {
       if(returnFlag == 1)
         changeRAngle(1);
       else if(returnFlag == 2)
         changeRAngle(-1);
         //subCycle = 1;
         Serial.print("\tRETURN1");
     }
     //delay(1);  // Add delay if required
     
     if(cycle == 1 && abs(currentAngle) < LEAST_COUNT)
     {
         cycle = 2;
        // currentAngle = 0;
     }
     
     if(cycle == 2)
     {
       //currentAngle = 0;
       returnFlag = 0;
       cycle = 0;
     }
  }
}


void _print()
{
    Serial.print(" SONAR-1 :"); Serial.print(dist[0]); Serial.print(" cm");
    Serial.print("\t SONAR-2 :"); Serial.print(dist[1]); Serial.print(" cm");
    Serial.print("\t CurrentAngle :"); Serial.print(currentAngle);
    //Serial.print("\t ADC Value :"); Serial.print(value); 
    Serial.print("\t obstacleFlag : "); Serial.print(obstacleFlag); 
    Serial.print("\t returnFlag : "); Serial.print(returnFlag); 
    Serial.print("\t cycle : "); Serial.print(cycle); 
    Serial.print("\t exec_time : "); Serial.println(exec_time); 
}

void initializeSonar()
{
  int m;
  
  for(m=0; m<NUM_OF_READINGS; m++)
  {
     distData1[m] = 1000;
     distData2[m] = 1000;
  } 
}

void setup()
{
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT); 
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(STOPPIN, OUTPUT);

  setPin(DIRPIN);
  noInterrupts();
  timInitPWM();
  tim3_Init();
  tim4_Init();
  interrupts(); // enable all(global) interrupts	
  Serial.begin(115200);
  Serial1.begin(115200);
  
  enableSonar(1);
  enableSonar(2);
  
  digitalWrite(STOPPIN, HIGH);
  
  initializeSonar();
}

void loop()
{		
  handleObstacle(1); 
  turn();
  planReturn();
  _print();   
 delay(50);
  time = millis();
  exec_time = time - prev_time;
  prev_time = time;  								
}

int serialEvent1() {
  while(Serial1.available())
  {
    // get the new byte:
    currentAngle = (int)Serial1.read(); 
    if(currentAngle >= 128)
    {
      currentAngle -= 255;
    }
    currentAngle = currentAngle;
    //Serial.print("\tCurrentAngle :"); Serial.println(currentAngle);
  }
}

