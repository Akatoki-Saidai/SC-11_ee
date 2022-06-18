#include <SPI.h>

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
  short X, Y, Z;
  float x, y, z;

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

  delay(10);
}
