#include<SPI.h>
#include<MadgwickAHRS.h>
Madgwick MadgwickFilter;
//地磁気から

#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;


//gyroよりコピー

const int L3GD20_CS = SS;
//const int SS = 10;      // 必ず 10 番を出力にすること
//const int MOSI = 11;
//const int MISO = 12;
//const int SCK  = 13;

const byte L3GD20_WHOAMI = 0x0f;
const byte L3GD20_CTRL1 = 0x20;
const byte L3GD20_CTRL2 = 0x21;
const byte L3GD20_CTRL3 = 0x22;
const byte L3GD20_CTRL4 = 0x23;
const byte L3GD20_CTRL5 = 0x24;
const byte L3GD20_X_L = 0x28;
const byte L3GD20_X_H = 0x29;
const byte L3GD20_Y_L = 0x2A;
const byte L3GD20_Y_H = 0x2B;
const byte L3GD20_Z_L = 0x2C;
const byte L3GD20_Z_H = 0x2D;

const byte L3GD20_RW = 0x80;
const byte L3GD20_MS = 0x40;

void L3GD20_write(byte reg, byte val)
{
  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(L3GD20_CS, HIGH);
}

byte L3GD20_read(byte reg)
{
  byte ret = 0;

  digitalWrite(L3GD20_CS, LOW);
  SPI.transfer(reg | L3GD20_RW);
  ret = SPI.transfer(0);
  digitalWrite(L3GD20_CS, HIGH);
  
  return ret;
}






void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)

  Serial.begin(115200);
  MadgwickFilter.begin(100);//100Hz

  while (!Serial) {}

  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4

  L3GD20_write(L3GD20_CTRL1, B00001111);
                         //   |||||||+ X axis enable
                         //   ||||||+- Y axis enable
                         //   |||||+-- Z axis enable
                         //   ||||+--- PD: 0: power down, 1: active
                         //   ||++---- BW1-BW0: cut off 12.5[Hz]
                         //   ++------ DR1-DR0: ODR 95[HZ]
 

  //地磁気から

   Serial.begin(115200);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
    delay(10);
}



void loop() {
  int axpin = A7, aypin = A6, azpin = A5;
  int mX = analogRead(axpin);
  int mY = analogRead(aypin);
  int mZ = analogRead(azpin);
  //単位を重力加速度(補正済み)
  float ax = (3*(mX-511.5)/511.5+0.92)/(-0.42);
  float ay = (3*(mY-511.5)/511.5+0.93)/(-0.41);
  float az = (3*(mZ-511.5)/511.5+0.94)/(-0.42);

  short X, Y, Z;
  float gx, gy, gz;//単位を°/s
  X = L3GD20_read(L3GD20_X_H);
  gx = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  gy = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  gz = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
 
  gx *= 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 0.07;  // +-2000dps
  gy *= 0.00875; // +-250dps
  gz *= 0.00875; // +-250dps


//地磁気から

  Vector mag = compass.readRaw();

  if(mag.XAxis>maxX){
    maxX=mag.XAxis;
  }
  if(mag.YAxis>maxY){
    maxY=mag.YAxis;
  }

  if(mag.XAxis<minX){
    minX=mag.XAxis;
  }
  if(mag.YAxis<minY){
    minY=mag.YAxis;
  }
  if(mag.ZAxis>maxZ){
    maxZ=mag.ZAxis;
  }
  if(mag.ZAxis<minZ){
    minZ=mag.ZAxis;
  }
  
  

  Vector norm = compass.readNormalize();
  
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-7.0 + (47.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;

  MadgwickFilter.update(gx,gy,gz,ax,ay,az,mag.XAxis,mag.YAxis,headingDegrees);
  float roll = MadgwickFilter.getRoll();
  float pitch = MadgwickFilter.getPitch();
  float yaw = MadgwickFilter.getYaw();

  Serial.print("roll:");
  Serial.println(roll);
  Serial.print("pitch:");
  Serial.println(pitch);
  Serial.print("yaw:");
  Serial.println(yaw);

  delay(10);
}
