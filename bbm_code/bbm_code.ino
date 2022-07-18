#include <Wire.h>
#include <SPI.h>
#include <SparkFunBME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <DFRobot_QMC5883.h>

//GNSS
SoftwareSerial serialConsole(1,2); //RX=1ピン, TX=2ピン
TinyGPSPlus gps;

//BME280
const int SPI_CS_PIN = 10;
BME280 sensor;

//ADXL375
int xpin = A0;
int ypin = A1;
int zpin = A2;
int acX, acY, acZ;
double gX, gY, gZ;

//QMC5883
DFRobot_QMC5883 compass;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

//gyro
short X, Y, Z;
float x, y, z;
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
  Serial.begin(115200);
  //setup for BMP280
  sensor.beginSPI(SPI_CS_PIN);
  //set up for GNSS at software serial
  serialConsole.begin(9600);
  //set up for QMC5883
    // Initialize Initialize QMC5883
    while (!compass.begin()){
      Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
      delay(500);
    }
    if(compass.isHMC() ){
      Serial.println("Initialize HMC5883");
      compass.setRange(HMC5883L_RANGE_1_3GA);
      compass.setMeasurementMode(HMC5883L_CONTINOUS);
      compass.setDataRate(HMC5883L_DATARATE_15HZ);
      compass.setSamples(HMC5883L_SAMPLES_8);
    }else if(compass.isQMC()){
      Serial.println("Initialize QMC5883");
      compass.setRange(QMC5883_RANGE_2GA);
      compass.setMeasurementMode(QMC5883_CONTINOUS); 
      compass.setDataRate(QMC5883_DATARATE_50HZ);
      compass.setSamples(QMC5883_SAMPLES_8);
    }
  //gyro
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)

  Serial.begin(9600);
  while (!Serial) {}

  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4

  L3GD20_write(L3GD20_CTRL1, B00001111);
                         //   |||||||+ X axis enable
                         //   ||||||+- Y axis enable
                         //   |||||+-- Z axis enable
                         //   ||||+--- PD: 0: power down, 1: active
                         //   ||++---- BW1-BW0: cut off 12.5[Hz]
                         //   ++------ DR1-DR0: ODR 95[HZ]
}

void loop() {
  while (serialConsole.available()){
    gps.encode(serialConsole.read());  
  }
  if (gps.location.isUpdated()){
    //time from GNSS
    Serial.print("time>>");
    Serial.print(gps.time.hour() + 9);
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print("\n");
    //latitude from GNSS
    Serial.print("lat>>");
    Serial.print(gps.location.lat(),6);
    Serial.print("\n");
    //longitude from GNSS
    Serial.print("lng>>");
    Serial.print(gps.location.lng(),6);
    Serial.print("\n");
    //temperature from BMP280
    Serial.print("temp>>");
    Serial.print(sensor.readTempC(),2);
    Serial.print("\n");
    //pressure from BMP280
    Serial.print("pres>>");
    Serial.print(sensor.readFloatPressure() / 100.0 ,1);
    Serial.print("\n");
    //accelation from ADXL375
      //電圧の値を0~1023で取得
      acX = analogRead(xpin);
      acY = analogRead(ypin);
      acZ = analogRead(zpin);
      //電圧値を加速度(単位は重力加速度)に変換
      gX = 3*(acX-511.5)/511.5;
      gY = 3*(acY-511.5)/511.5;
      gZ = 3*(acZ-511.5)/511.5;
      //誤差補正
      Serial.print("acX>>");
      Serial.println((gX+0.92)/(-0.42));
      Serial.print("\n");
      Serial.print("acY>>");
      Serial.println((gY+0.93)/(-0.41));
      Serial.print("\n");
      Serial.print("acZ>>");
      Serial.println((gZ+0.94)/(-0.42));
      Serial.print("\n");
    //magnetism from QMC5883
    Vector mag = compass.readRaw();
    Serial.print("magX:");
    Serial.print(mag.XAxis-1000);
    Serial.print("\n");
    Serial.print("magY");    
    Serial.print(mag.YAxis-1000);
    Serial.print("\n");
    Serial.print("magZ:");
    Serial.print(mag.ZAxis-1000);
    Serial.print("\n");
    //gyro
    X = L3GD20_read(L3GD20_X_H);
  x = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  y = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  z = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
 
  x *= 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 0.07;  // +-2000dps
  y *= 0.00875; // +-250dps
  z *= 0.00875; // +-250dps
 
  Serial.print(X);    // X axis (reading)
  Serial.print("\t");
  Serial.print(Y);    // Y axis (reading)
  Serial.print("\t");
  Serial.print(Z);    // Z axis (reading)
  Serial.print("\t");
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)
  }
  delay(1000);
}
