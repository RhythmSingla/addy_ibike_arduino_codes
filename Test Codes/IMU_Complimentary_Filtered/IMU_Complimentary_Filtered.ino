#include <Wire.h>
#include <math.h>

#define PI_APP 3.14159265359
#define dt 0.1
#define G_SENS 131

const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, gyro_x, gyro_y;

double ratio, theta_y, theta_x, ang_x, ang_y, theta_y_old = 0, theta_x_old = 0;

void getMPUdata()
{
 Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  ang_x = atan2((double)AcY, (double)AcZ)*180/PI_APP;
  ang_y = atan2((double)AcX, (double)AcZ)*180/PI_APP;
  
  gyro_x = GyX/G_SENS;
  gyro_y = -GyY/G_SENS;
  
  theta_y = 0.85*(theta_y_old + gyro_y*dt) + 0.15*ang_y;
  theta_y_old = theta_y;
  
  theta_x = 0.85*(theta_x_old + gyro_x*dt) + 0.15*ang_x;
  theta_x_old = theta_x; 
}

void printData()
{
  //Serial.print("  Omega_X = "); Serial.print(gyro_x);
  Serial.print(" | Theta_X = "); Serial.print(theta_x);
 
  //Serial.print(" | Omega_Y = "); Serial.print(gyro_y);
  Serial.print(" | Theta_Y = "); Serial.println(theta_y); 
}

void setup()
{
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(38400);
}

void loop()
{
 getMPUdata();
 printData(); 

}
