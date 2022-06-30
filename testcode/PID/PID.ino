#include <SPI.h>

//PID制御の定義

float Kp=2      //比例ゲインKp
float Ki=120    //比例ゲインki
float Kd=1      //比例ゲインkd
float target=0         //目標角度[rad]

//bool LED;
//float duty = 0;
//float dt, preTime;
//float vol;
//float P, I, D, U, preP;

//モータドライバーの設定
//MotorDriver motor_driver();

//角速度センサの設定
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
 
  //Serial.print(X);    // X axis (reading)
  //Serial.print("\t");
  //Serial.print(Y);    // Y axis (reading)
  //Serial.print("\t");
  //Serial.print(Z);    // Z axis (reading)
  //Serial.print("\t");
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.println(z);  // Z axis (deg/sec)

  //PID制御のプログラム
  static float integral=0; //Iの設定
  static float last_err=0; //偏差
  static unsigned long last_micros=0;

  float current_rad=x; //現在の角度（ロー角）を取得
  float err=current_rad-target;

  float err=abs(err); //errを絶対値に変換
  
  float P=kp*err; //Pを計算
  unsigned long current_micros=micros(); //現在の時間を取得
  float dt=((float)(current_micros-last_micros))/1000000.0; //経過時間を取得

  integral+=err*dt; //偏差の積分
  float I=ki*integral; //Iを計算
  float diff=(err-last_err)/dt; //偏差の微分を計算
  float D=kd*diff; 
  
  //motor_driver.setMotorSpeed(P+I+D); PIDでモータ制御
  if(err>10){
    println("モータ回転");
  }else{
    println("安定姿勢");
  }
  last_err=err; //現在の偏差を保存
  last_micros=current_micros; //現在の時間を保存

  delay(100);
}




//#define Kp      2
//#define Ki      120
//#define Kd      1
//#define target  

//bool LED;
//float duty = 0;
//float dt, preTime;
//float vol;
//float P, I, D, U, preP;

//void setup() {
// Serial.begin(115200);
// delay(1000);
//}

//void loop() {
// for (int i = 0; i < 1000; i++) {
//    vol += analogRead(0);
//  }
//  vol = 5.0 * (vol / 1000) / (1 << 10);

//  PID();
//  analogWrite(3, duty);

  //Serial.print(dt , 3); Serial.print(",");
  //Serial.print(duty, 3); Serial.print(",");
  //Serial.print(P, 3); Serial.print(",");
  //Serial.print(I, 3); Serial.print(",");
  //Serial.print(D, 3); Serial.print(",");
 // Serial.println(vol, 3);
//}

//inline void PID() {
//  dt = (micros() - preTime) / 1000000;
//  preTime = micros();
//  P  = target - vol;
//  I += P * dt;
//  D  = (P - preP) / dt;
//  preP = P;

 // U = Kp * P + Ki * I + Kd * D;
 // if (U > 255)U = 255;
 // if (U < 0)  U = 0;
 // duty += (U - duty) * 0.2;
  //duty = U;
//}
