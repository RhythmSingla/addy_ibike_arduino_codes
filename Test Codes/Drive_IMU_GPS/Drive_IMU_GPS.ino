#include <Wire.h>
#include <math.h>

#define PI_APP 3.14159265359
#define G_SENS 131
#define dt 0.00001

#define SPEEDPIN 2
#define STOPPIN 23

/* IMU raw Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int count = 0;
uint32_t timer;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleZ0; // Base angle
double compAngleZ; // Calculated angle using a complementary filter
volatile int average = 0;
double yaw;
double PreviousYaw = 0;

const int MPU = 0x68;  // I2C address of the MPU-6050

/**** Variables for moving average filter ****/
const int numReadings = 10;
int readings[numReadings];      // complementary filter angles
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
double AVG_yaw = 0;

char temp,longi[11],lati[11];
int i;
float LATI = 0,LONGI = 0;
float g_distance = 0;
float g_initial_lati = 0, g_initial_longi = 0;
int g_first_data = 0;
int var = 5;


// Calculates distance between (gpsLat0,gpsLong0) and (gpsLat,gpsLong) 

void Distance(float gpsLat0, float gpsLong0, float gpsLat, float gpsLong)
{
  float delLat = abs(gpsLat0-gpsLat)*111194.9;
  float delLong = 111194.9*abs(gpsLong0-gpsLong)*cos(radians((gpsLat0+gpsLat)/2));
  g_distance = sqrt(pow(delLat,2)+pow(delLong,2));
  if (g_first_data >= var)
    {
      Serial.print("Distance :"); Serial.print(g_distance);
      Serial.print("For  "); Serial.print(gpsLat0); Serial.print(gpsLong0); Serial.print(gpsLat);Serial.print(gpsLong);
    }
//  return g_distance;
}


void getlatilongi(void)
{                                      //==================== searching for "GG" ===================//
     while ( !Serial2.available() );    
        while ( 'G' != Serial2.read() )
        {
           while(!Serial2.available());
        }        
      
        while(!Serial2.available());
     while ( 'G' != Serial2.read() )
     { 
       while ( !Serial2.available() );    
        while ( 'G' != Serial2.read() )
        {
           while(!Serial2.available());
        }        
      
        while(!Serial2.available());
     }
    

 /***************************************************************************************************************************/
                                                                          //============== seeking for north cordi
    
    
        while (!Serial2.available() );                           
     while ( ',' != Serial2.read() )                        //this flushes all values between two commas
     {
         while ( !Serial2.available() );
     }
 
    while ( !Serial2.available() );                           
     while ( ',' != Serial2.read() )
     {
         while ( !Serial2.available() );
     }
 
/****************************************************************************************************************************/
   // Serial.print(" N: ");
    
   
        while ( !Serial2.available() ); 
       
        for( i=0,temp=Serial2.read();',' !=temp;i++)                //storing all north latitude values
        {
          lati[i]=temp;
          while ( !Serial2.available() );
          temp=Serial2.read();
          
        }
        lati[i]='\0';
        float m=atof(lati);                                        //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
        //Serial.print(m,8);                          
       // Serial.print("   ");
      LATI=(int)(m/100)+(m-((int)(m/100))*100)/60.0;
      //Serial.print(LATI,8);
/********************************************************************************************************************************/
    
    
        while ( !Serial2.available() );                       // reading a character from the GPS
     while ( ',' != Serial2.read() )
     {
    while ( !Serial2.available() );   
     }

/***************************************************************************************************/

    // Serial.print(" E: ");
    while ( !Serial2.available() ); 
       
        for( i=0,temp=Serial2.read();',' !=temp;i++)
        {
          longi[i]=temp;
          while ( !Serial2.available() );
          temp=Serial2.read();
          
        }
        longi[i]='\0';
        float n=atof(longi);                                              //conversion of array to floating value that is RAW data of GPS and then converting it in degrees
        //Serial.print(m,8);
        //Serial.print("   ");
      LONGI=(int)n/100+(n-((int)n/100)*100)/60.0;
      //Serial.print(LONGI,8);
   if (g_first_data < var && LATI != 0 && LONGI != 0)
    {
        g_first_data++;
        g_initial_lati = LATI;
        g_initial_longi = LONGI;
        Serial.print("First data Receieved");   
    }
   
   if(LATI != 0 && LONGI != 0 && g_first_data == var)
     Distance(g_initial_lati, g_initial_longi, LATI, LONGI);
//if (g_first_data)
//    Serial.print("Distance : "); Serial.print(Distance(g_initial_lati, g_initial_longi, LATI, LONGI));
  
}


void gpsInit()
{
  for(int i=0;i<10;i++)                  //initializing total array zero string to avoid producing garbage values
  {
    lati[i]='0';
    longi[i]='0';
  
  }
     lati[10]='\0';
     longi[10]='\0';
     delay(100);  
}

void printGPSData()
{
  Serial.print("Latitude: ");
  Serial.print(LATI,8);
  Serial.print("\tLongitude: ");
  Serial.print(LONGI,8); 
  Serial.println(); 
}

void getMPUdata()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  accX   = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY    = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ    = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX     = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY     = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ     = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  /* yaw angle from accelerometer */
  yaw = atan2(-1*(double)accY, (double)accX) * 180 / PI_APP;
}

double getyaw()
{
  yaw = (compAngleZ0 - compAngleZ)*PI_APP/180;
  yaw = atan2(sin(yaw),cos(yaw))*180/PI_APP;
  
#if 0
  Serial.print(" yaw : "); Serial.print(yaw); Serial.print("\t");
  Serial.print(" Base angle : "); Serial.print(compAngleZ0); Serial.print("\t");
  Serial.print(" compAngle : "); Serial.print(compAngleZ); Serial.print("\t");    
  Serial.print("\t");
#endif

  return yaw;
}

void controlDrive()
{                                                                                                                                                                                                                                                                                             
 if(millis() < 40000 && digitalRead(STOPPIN) == HIGH)
 {
    analogWrite(SPEEDPIN, 82);
 } 
 else
 {
    analogWrite(SPEEDPIN, 78);
 } 
}

void sendYaw()
{
  Serial1.write((int)average);  
}

void getHeadingAngle()
{
 /* Update all values */
  getMPUdata();

  /* Calculation of omega from gyro */
  double gyroZrate =  gyroZ / G_SENS;

  /* Calculate Zangle using complimentary filter */
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  /* Calculate yaw after 100th reading onwards*/
  if (count == 100)
    compAngleZ0 = compAngleZ;
  if (count >= 100)
  {    
    double currentYaw = getyaw();   
    Serial.print("currentYaw : "); Serial.print(currentYaw); Serial.print("\t"); 
    
    #if 0
    Serial.print("currentYaw : "); Serial.print(currentYaw); Serial.print("\t");
    Serial.print(" Previous yaw : "); Serial.print(PreviousYaw);  Serial.print("\t");
    #endif
    
    //subtract the last reading
    total = total - readings[readIndex];
    //take new reading
    readings[readIndex] = currentYaw;
    
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
     }
    // calculate the average:
    average = total / numReadings;
    // send it to the computer as ASCII digits
    PreviousYaw = average;
    
    Serial.print("Average yaw:"); Serial.println(average);
    sendYaw();
    delay(20);        // delay in between reads for stability
  }  
  count++;
  if (count >= 150)
    count = 101; 
}

void setup()
{
  pinMode(STOPPIN, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  gpsInit(); 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);

  /* set gyro starting angle from accelerometer */
  getMPUdata();
  compAngleZ = yaw;

  /* Initialize the moving average reading */
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;}   
 // Serial.println("here");  
  
  pinMode(SPEEDPIN, OUTPUT);
  analogWrite(SPEEDPIN, 0);
  delay(2000);
  analogWrite(SPEEDPIN, 78);
}



void loop()
{
  printGPSData();
  getlatilongi(); 
  getHeadingAngle();
  controlDrive();
  //delay(50);
  //Serial.println("here");
}
