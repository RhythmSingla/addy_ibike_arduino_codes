#include <Wire.h>
#include <math.h>
#include "compass.h"

#define PI_APP 3.14159265359
#define dt_s 0.00001
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead 

#define SPEEDPIN 2
#define STOPPIN 23

const int MPU_H = 0x69;  // HANDLE/SEAT  IMU set AD0 to logic high for 0x69
const int MPU_S = 0x68;  // STEER IMU 
const int PCF8591 = 0x48;  // Drive motor DAC module address

/* IMU raw Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint32_t timer;

/* Roll and pitch calculated using the accelerometer 
 * while yaw calculated using magnetometer/ accelerometer */
double roll, pitch, yaw; 
//For Seat IMU
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter

//For steer IMU
double compAngleSteer0; // Base angle
double compAngleSteer; // Calculated angle using a complementary filter
int steer_angle_count = 0;  // keeps count 

volatile int averageHandleAngle = 0;
double PreviousSteer = 0;

/**** Variables for moving average filter ****/
const int numReadings = 10;
int readings[numReadings];      // complementary filter angles
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
double AVG_yaw = 0;
/****************************************************/
char temp,longi[11],lati[11];
int i;
float LATI = 0,LONGI = 0;
float g_distance = 0;
float g_initial_lati = 0, g_initial_longi = 0;
int g_first_data = 0;
int gps_count = 5; // neglect first gps_count data of gps

/*******************************************************/
String inputString2="";
String wantedstring= "";
bool stringComplete2=false;
bool flag=false;
float temp_lati;
float temp_longi;
String inputString1 = "";         // a string to hold incoming data
boolean stringComplete1 = false;  // whether the string is complete
String lati_str = "", longi_str = "";
float target_lati = 0, target_longi = 0;

/****************************************************/

/*************************************** GPS funcitons ******************************************************/

// Calculates distance between (gpsLat0,gpsLong0) and (gpsLat,gpsLong) 
void Distance(float gpsLat0, float gpsLong0, float gpsLat, float gpsLong)
{
  float delLat = abs(gpsLat0-gpsLat)*111194.9;
  float delLong = 111194.9*abs(gpsLong0-gpsLong)*cos(radians((gpsLat0+gpsLat)/2));
  g_distance = sqrt(pow(delLat,2)+pow(delLong,2));
  if (g_first_data >= gps_count)
    {
      Serial.print("Distance :"); Serial.print(g_distance); Serial.print("\t");
//      Serial.print("For  "); Serial.print(gpsLat0); Serial.print(gpsLong0); Serial.print(gpsLat);Serial.print(gpsLong);
    }
}


void serialEvent2()
{
  while(Serial2.available()>0)
  {
    char inchar = (char)Serial2.read();
    inputString2+= inchar;
     
     if(inputString2 == "$GPGGA" || flag == true )
     {
       flag = true;
       wantedstring+=inchar;
       if( inchar == '\n' )
        {
        stringComplete2 = true;
        flag = false;
       }
     }
     
     if(inchar == '\n')
     inputString2 = "";     
    
   }
}

void getlatilongi(String input)
{                                   
  int i,j;
  for( i =0 ; input[i] != ',' ; i++)
  {
  }
  i++;
  
  for ( j=0 , i>0 ; input[i] != ',' ; i++ , j++)
  {    
  }
  i++;
  
  for ( j=0 , i> 0 ; input[i] != ',' ; i++ , j++)
  {
    lati[j]=input[i];
  }
  
  lati[j+1] = '\0';
  i++;
  
  for ( j=0 , i>0 ; input[i] != ',' ; i++ , j++)
  {    
  }
  i++;
  
  
  for ( j=0 , i> 0 ; input[i] != ',' ; i++ , j++)
  {
    longi[j]=input[i];
  }
  
  longi[j+1] = '\0';
  
  
  temp_lati=atof(lati);
  temp_longi=atof(longi);
  
  LATI=(int)(temp_lati/100) + (temp_lati-((int)(temp_lati/100))*100)/60.0;
  LONGI=(int)(temp_longi/100) + (temp_longi-((int)(temp_longi/100))*100)/60.0;
 
   if (g_first_data < gps_count && LATI != 0 && LONGI != 0)
    {
        g_first_data++;
        g_initial_lati = LATI;
        g_initial_longi = LONGI;
        Serial.print("First data Receieved");   
    }
   
   if(LATI != 0 && LONGI != 0 && g_first_data == gps_count && target_lati != 0 && target_longi != 0)
     Distance(target_lati, target_longi, LATI, LONGI);
  
}


/*********************************************************************************/

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
}

void gps_conditioning()
{
   if (stringComplete2 == true)
  {
   
    getlatilongi(wantedstring);
    stringComplete2 = false;
    wantedstring = "";
    printGPSData();

  }
}

void parse1()
{
  int len = inputString1.length();
  int i=0, j=0, k=0;
  
  for(i=0; inputString1[i] != ','; i++)
  {
    lati_str += inputString1[i];  
  }
  i++;
  lati_str[i] = '\0';
  
  for(j=0; inputString1[i] != '$'; i++,j++)
  {
    longi_str += inputString1[i];  
  }
  i++;
  j++;
  longi_str[j] = '\0';
  
  target_lati = lati_str.toFloat();
  target_longi = longi_str.toFloat();
}

void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // add it to the inputString:
    inputString1 += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '$') {
      stringComplete1 = true;
    } 
  }
}

void _print()
{
      //Serial.print("Lati_from_PI : "); Serial.print(lati_str);
      //Serial.print("\t Longi_from_PI : "); Serial.print(longi_str);
      
      Serial.print("\t Target Lati : "); Serial.print(target_lati,8);
      Serial.print("\t Target Longi : "); Serial.println(target_longi,8);  
}

void handleGPSData()
{
   // print the string when a newline arrives:
  if (stringComplete1) {
   // Serial.println(inputString1); 
    // clear the string:
    parse1();
    _print();
    inputString1 = "";
    lati_str = "";
    longi_str = "";
    stringComplete1 = false;
  }
}


/***********************************************************END GPS function **************************************/

/*********************************************************** Motor drive function ********************************/
void controlDrive(int digitalSpeed)
{                                                                                                                                                                                                                                                                                             
 Wire.beginTransmission(PCF8591); // wake up PCF8591
 Wire.write(0x40); // control byte - turn on DAC (binary 1000000)
 Wire.write(digitalSpeed); // value to send to DAC
 Wire.endTransmission(); // end tranmission
}
/*********************************************************** END Motor drive function ********************************/

/*********************************************************** IMU functions *******************************************/
void getMPUdata(int address)
{
  Wire.beginTransmission(address);
  Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);          // request a total of 14 registers
  accX    = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY    = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ    = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX   = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY   = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ   = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

void getRPY(int address)
{
  getMPUdata(address);
if (address == MPU_S)
  yaw = atan2(-1*(double)accY, (double)accX) * 180 / PI_APP;
  
else
{
#ifdef RESTRICT_PITCH 
  roll  = atan2(accY, accZ) * 180/PI_APP;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180/PI_APP;

#else 
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180/PI_APP;
  pitch = atan2(-accX, accZ) * 180/PI_APP;

#endif

#ifdef ACCEL_YAW 
  yaw = atan2(-1*(double)accY, (double)accX) * 180 / PI_APP;

#endif
} 
}

double MagYaw()
{
  compass_scalled_reading();
  
  double rollAngle  = compAngleX * PI_APP/180;
  double pitchAngle = compAngleY * PI_APP/180;

  double Bfy = compass_z_scalled * sin(rollAngle) - compass_y_scalled * cos(rollAngle);
  double Bfx = compass_x_scalled * cos(pitchAngle) + compass_y_scalled * sin(pitchAngle) * sin(rollAngle) + compass_z_scalled * sin(pitchAngle) * cos(rollAngle);
  double  magYaw = atan2(-Bfy, Bfx) * 180/ PI_APP;

//  Serial.print("magYaw : "); Serial.print(magYaw); Serial.print('\t');
  return magYaw;
  
}

double transformAngle(double baseAngle, double currentAngle)
{
  double angle = (baseAngle - currentAngle)*PI_APP/180;
  angle = atan2(sin(angle),cos(angle))*180/PI_APP;
  
#if 0
  Serial.print(" Angle : "); Serial.print(angle); Serial.print("\t");
  Serial.print(" Base angle : "); Serial.print(baseAngle); Serial.print("\t");
  Serial.print(" Current angle : "); Serial.print(currentAngle); Serial.print("\t");    
#endif

  return angle;
}

/*** Heading IMU update ***/
void updateRollPitchYaw() {

  getRPY(MPU_H);  

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convter to deg/s

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
 
/********************  Complementary filter   **********************/
/* compAngel + gyrorate*dt = gyro part */
/* roll, pitch, yaw = accelerometer part */

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
 
#ifdef ACCEL_YAW 
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
  
#else 
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * MagYaw();
  
#endif

#if 1
  //Serial.print("Roll : ");Serial.print(compAngleX); Serial.print('\t');
  //Serial.print("Pitch : "); Serial.print(compAngleY); Serial.print('\t');
  Serial.print("Heading : "); Serial.print(compAngleZ);
#endif

}


/*** Steer IMU angle update ***/
void updateSteerAngle()
{
  /* Update all values */
  getRPY(MPU_S);
  
  /* Calculation of omega from gyro */
  double gyroZrate =  gyroZ / 131.0;

  /* Calculate Zangle using complimentary filter */
  compAngleSteer = 0.93 * (compAngleSteer + gyroZrate * dt_s) + 0.07 * yaw;

  /* Calculate yaw after 100th reading onwards*/
  if (steer_angle_count == 100)
    compAngleSteer0 = compAngleSteer;
  if (steer_angle_count >= 100)
  {    
    double currentSteer = transformAngle(compAngleSteer0, compAngleSteer);   
    //Serial.print("currentSteer : "); Serial.print(currentSteer); Serial.print("\t"); 
    
    #if 0
    Serial.print("currentSteer : "); Serial.print(currentSteer); Serial.print("\t");
    Serial.print(" Previous Steer : "); Serial.print(PreviousSteer);  Serial.print("\t");
    #endif
    
    //subtract the last reading
    total = total - readings[readIndex];
    //take new reading
    readings[readIndex] = currentSteer;
    
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
     }
    // calculate the average:
    averageHandleAngle = total / numReadings;
    // send it to the computer as ASCII digits
    PreviousSteer = averageHandleAngle;
    
    Serial.print("  Handle Angle:"); Serial.println(averageHandleAngle);
  }  
  steer_angle_count++;
  if (steer_angle_count >= 150)
    steer_angle_count = 101; 
}

void sendAngleDatatoSteer()
{
  Serial1.write(255);
  Serial1.write((int)averageHandleAngle); 
  Serial1.write((int)compAngleZ);
  Serial1.write(254);
}

/*********************************************************** END IMU functions ***************************************/

void setup()
{
  pinMode(STOPPIN, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  
  inputString1.reserve(50);
  lati_str.reserve(12);
  longi_str.reserve(12);
  
  Wire.begin();
  
  /*********************** Seat IMU routine ************************/
  /* Rotate the IMU "Takes some time" */ 
  /* Magnetometer default parameters */
  compass_x_offset = -785.89;
  compass_y_offset = 1243.25;
  compass_z_offset = 751.30;
  compass_x_gainError = 8.39;
  compass_y_gainError = 8.67;
  compass_z_gainError = 8.17;
//compass_x_offset = -1421.94;
//compass_y_offset = 1487.46;
//compass_z_offset = 207.69;
//compass_x_gainError = 8.39;
//compass_y_gainError = 8.67;
//compass_z_gainError = 8.17;
//  
//  /*** Initialize and Calibrate Magnetometer ***/
//  /* compass_offset_calibration(0) 
//     Argument 
//              " 0 - Use default offset and gain values
//              " 1 - Calibrate only for gain values
//              " 2 - Calibrate only for offset values
//              " 3 - Calibrate for both gain and offset values
//   */
  compass_init(5);
  compass_debug = 1;
  compass_offset_calibration(0);
// 
//  /*** Initialize MPU ***/
  Wire.beginTransmission(MPU_H);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);

  /* Get RPY from SEAT IMU */
  getRPY(MPU_H);
  /* set gyro starting angle from accelerometer and magnetometer */
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = MagYaw();
  timer = micros();
 /*********************** END Seat IMU routine ************************/ 
 /*********************** GPS init routine ************************/
  gpsInit(); 
 /*********************** END GPS init routine ************************/ 
 /*********************** STEER IMU routine ************************/
  Wire.beginTransmission(MPU_S);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);

  /* set gyro starting angle from accelerometer */
  getRPY(MPU_S);
  compAngleSteer = yaw;

  /* Initialize the moving average reading */
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;}
/*********************** END STEER IMU routine ************************/
/*********************** Drive motor routine **************************/ 

 /*********************** END Drive motor routine ************************/ 
 
}

void loop()
{
  updateRollPitchYaw();
  updateSteerAngle();
  sendAngleDatatoSteer();
  //gps_conditioning();
  handleGPSData();
//  if (abs(g_distance) > 5 && LATI != 0 && LONGI != 0 && target_lati != 0 && target_longi != 0)
//    controlDrive(80);
//  else
//    controlDrive(0);
  //Serial.println();
}
