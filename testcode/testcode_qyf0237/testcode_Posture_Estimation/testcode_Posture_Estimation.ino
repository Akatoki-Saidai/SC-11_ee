#include<SPI.h>
#include<MadgwickAHRS.h>
Madgwick MadgwickFilter;
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
int setupco = 0;
float sumgx, sumgy, sumgz;
int count = 0;
long endtime;
short em_time=0;
float em_roll=0.0;
float em_pitch=0.0;
float em_yaw=0.0;
bool error_modifying = true;

//姿勢制御追加
float Kp = 5;         // 比例ゲインKp
float Ki = 200;       // 比例ゲインKi
float Kd = 300;       // 比例ゲインKd
float target = 0;     // 目標角度[rad]

//RL_motorから
#include <Servo.h>

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define ESC_PIN_R 4  //ESCへの出力ピン
#define ESC_PIN_L 5
int volume;  //可変抵抗の値を入れる変数
char message[50];  //シリアルモニタへ表示する文字列を入れる変数

Servo escR;
Servo escL;




void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)
  //9600だと100Hzに間に合わないかもしれない。
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



  //RL_motorから
  escR.attach(ESC_PIN_R);
  escL.attach(ESC_PIN_L);
  
  Serial.println("Writing maximum output.");
  
  escR.writeMicroseconds(MAX_SIGNAL);
  escL.writeMicroseconds(MAX_SIGNAL);
  
  Serial.println("Wait 2 seconds.");
  delay(4000);
  Serial.println("Writing minimum output");
  
  escR.writeMicroseconds(MIN_SIGNAL);
  escL.writeMicroseconds(MIN_SIGNAL);
  
  Serial.println("Wait 2 seconds. Then motor starts");
  delay(4000);
  delay(10);
}
void loop() {
  int axpin = A2, aypin = A1, azpin = A0;
  short mX = analogRead(axpin);
  short mY = analogRead(aypin);
  short mZ = analogRead(azpin);
  //単位を重力加速度(補正済み)
  float ax = (3*(mX-511.5)/511.5+0.92)/(-0.42);
  float ay = (3*(mY-511.5)/511.5+0.93)/(-0.41);
  float az = (3*(mZ-511.5)/511.5+0.94)/(-0.42);
  //確認用(加速度)
  /*Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.println(az);*/
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
  //確認用(角速度)
  /*Serial.print(gx+1.05);
  Serial.print(",");
  Serial.print(gy+1.9);
  Serial.print(",");
  Serial.println(gz-1.8);*/
  //角速度誤差補正
  if(setupco == 0){
    endtime = millis() + 1000;
    setupco = 1;
  }else if(setupco == 1){
    if(endtime >= millis()){
      sumgx += gx;
      sumgy += gy;
      sumgz += gz;
      count += 1;
    }else{
      sumgx = sumgx/count;
      sumgy = sumgy/count;
      sumgz = sumgz/count;
      setupco = 2;
    }
  }else if(setupco == 2){}
  //確認用(角速度誤差補正)
  /*Serial.print(gx - sumgx);
  Serial.print("\t");
  Serial.print(gy - sumgy);
  Serial.print("\t");
  Serial.println(gz - sumgz);*/
  //６軸姿勢推定　MadgwickFilter.updateIMU(角速度x,角速度y,角速度z,加速度x,加速度y,加速度z);
  MadgwickFilter.updateIMU(gy-sumgy,(-1)*(gx-sumgx),gz-sumgz,ax,ay,az);
  /*float roll = MadgwickFilter.getRoll();
  float pitch = MadgwickFilter.getPitch();
  float yaw = MadgwickFilter.getYaw();
  Serial.print("roll:");
  Serial.println(roll);
  Serial.print("pitch:");
  Serial.println(pitch);
  Serial.print("yaw:");
  Serial.println(yaw);*/
  //processing用コード
  float roll = MadgwickFilter.getRollRadians();
  float pitch = MadgwickFilter.getPitchRadians();
  float yaw = MadgwickFilter.getYawRadians();
  //ラジアン取得
  //誤差修正(20秒)
  if(error_modifying){
    em_time += 10;
    if(em_time > 20000/*20秒*/){
      em_roll = roll;
      em_pitch = pitch;
      em_yaw = yaw;
      error_modifying = false;}
   }
  /*Serial.print("e,");//データ識別用
  Serial.print(roll-em_roll);
  Serial.print(",");
  Serial.print(pitch-em_pitch);
  Serial.print(",");
  Serial.println(yaw-em_yaw);*/
  /*Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);*/
  //100Ｈｚに合わせる





  //姿勢制御追加
  if(em_time>20000){
  static float integral = 0;
  static float last_err = 0;
  static unsigned long last_micros = 0;
  
  float current_rad = pitch-em_pitch ;                // センサーから現在の角度を取得
  float err = target - current_rad;                   // 偏差を計算
  float P = Kp * err;                                 // Pを計算
  unsigned long current_micros = micros();            // 現在の時間を取得
  float dt = ((float)(current_micros - last_micros))
              / 1000000.0;                            // 経過時間を計算
  integral += err * dt;                               // 偏差の積分を計算
  float I = Ki * integral;                            // Iを計算
  float diff = (err - last_err) / dt;                 // 偏差の微分を計算
  float D = Kd * diff;                                // Dを計算


  //RL_motorから
  escR.writeMicroseconds(P+D+I);
  escL.writeMicroseconds(P+D+I);                      // PIDでモータを制御


  //以下数値は仮置き
  //右に流されたときの修正
  if(roll-em_roll>3.14/*傾き（ラジアン表記）*/){
    //現在の時間を取得
    unsigned long current_micros = micros(); 
    while(micros()<current_micros + 1000000/*出力時間*/){
        //機体をyaw方向に+90度回転させる。
        escR.writeMicroseconds(2.5/*入力電圧値*/);
    }
    while(micros()<current_micros + 1000000/*出力時間*/){
        //プロペラを左右回してroll方向の傾きを修正する。    
        escR.writeMicroseconds(2.5/*入力電圧値*/);
        escL.writeMicroseconds(2.5/*入力電圧値*/);
    }    
  }
    //左に流された時の修正
    if(roll-em_roll>3.14/*傾き（ラジアン表記）*/){
    //現在の時間を取得
    unsigned long current_micros = micros(); 
    while(micros()<current_micros + 1000000/*出力時間*/){
        //機体をyaw方向に+90度回転させる。
        escR.writeMicroseconds(2.5/*入力電圧値*/);
    }
    while(micros()<current_micros + 1000000/*出力時間*/){
        //プロペラを左右回してroll方向の傾きを修正する。    
        escR.writeMicroseconds(2.5/*入力電圧値*/);
        escL.writeMicroseconds(2.5/*入力電圧値*/);
    }
  }



  //test
 /* Serial.print(P);
  Serial.print(",");
  Serial.print(D);
  Serial.print(",");
  Serial.println(I);
  Serial.print(",");*/
  Serial.println(P+D+I);
  Serial.print(",");
  Serial.println(pitch-em_pitch);
  Serial.println(roll-em_pitch);
  
  last_err = err;                                     // 現在の偏差を保存
  last_micros = current_micros;                       // 現在の時間を保存
 
  
  }
  
  delay(10);
}
