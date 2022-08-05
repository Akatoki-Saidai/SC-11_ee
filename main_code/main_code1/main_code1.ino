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


//コピペ
#include <Servo.h>
#include <TinyGPS++.h>
#include <DFRobot_QMC5883.h>
#include <math.h>


#define rad2deg(a) ((a) / M_PI * 180.0) /* rad を deg に換算するマクロ関数 */
#define deg2rad(a) ((a) / 180.0 * M_PI) /* deg を rad に換算するマクロ関数 */

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define ESC_PIN_R 4  //ESCへの出力ピン
#define ESC_PIN_L 5

//PID制御のためのプログラム
float err;//角度の偏差

int phase = 1;

DFRobot_QMC5883 compass;
double heading;
double declinationAngle;
double headingDegrees;
double CurrentDistance;
double Angle_gy270;
double Angle_Goal;
double Angle_gps;
double Angle_heading;
double rrAngle, llAngle;
double sum_latitude = 0;
double sum_longitude = 0;
double GOAL_lat = 35.860545000;
double GOAL_lng = 139.606940001;
double desiredDistance = 10.0; //Mode-Fに移行できる距離
// variables___GPS
//緯度
double GPSlat_array[5];
double GPSlat_sum = 0;
double GPSlat_data;
//経度
double GPSlng_array[5];
double GPSlng_sum = 0;
double GPSlng_data;
double delta_lng;
//距離
double distance; //直進前後でゴールに近づいているかどうかを表す
double Pre_distance;
// variables___GY-271
double heading_data;
double heading_array[5];
double heading_sum = 0;
double omega;
double azimuth;
double gps_latitude, gps_longitude;
//for Mode-B
//int CalibrationCounter = 1;
//int calibration = 1;
//int sum_count = 0;
//double CurrentDistance;

//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{
  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);
  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);
  double EarthRadius = 6378.137; 
  //目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;
  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;
  return distance;
}

//角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{
  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);
  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);
  //目標地点までの角度を導出
  delta_lng = GOAL_lng - gps_longitude;
  azimuth = rad2deg(atan2(sin(delta_lng), cos(gps_latitude) * tan(GOAL_lat) - sin(gps_latitude) * cos(delta_lng)));
  if (azimuth < 0)
  {
    azimuth += 360;
  }
  else if (azimuth > 360)
  {
    azimuth -= 360;
  }
  return azimuth;
}

//機体の向き(heading)の計算
double CalculateHeading(double heading)
{
  double Sum_headingDegrees=0;
  declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
  heading+=declinationAngle;
    if (heading < 0){
    heading += 2 * PI;
    }
    if (heading > 2 * PI){
    heading -= 2 * PI;
    }
    // Convert to degrees
    headingDegrees = heading * 180 / M_PI;
    if (headingDegrees < 0){
    headingDegrees += 360;
    }
    if (headingDegrees > 360){
    headingDegrees -= 360;
    }
    for(int i=0;i<15;i++){
    Sum_headingDegrees += headingDegrees;
    }
    return (Sum_headingDegrees/15);             
}
//モータを動かすプログラム
int volume;  //可変抵抗の値を入れる変数
char message[50];  //シリアルモニタへ表示する文字列を入れる変数
Servo escR;
Servo escL;

//右の回転数を上げる
void RightRotating(){
  escR.writeMicroseconds(1800);
  escL.writeMicroseconds(1300);
}
//左の回転数を上げる
void LeftRotating(){
  escR.writeMicroseconds(1300);
  escL.writeMicroseconds(1800);
}




void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 8MHz/8 = 1MHz; (max 10MHz)
  //9600だと100Hzに間に合わないかもしれない。
  Serial.begin(115200);
  MadgwickFilter.begin(100);//100Hz

  Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show D4
  L3GD20_write(L3GD20_CTRL1, B00001111);
                         //   |||||||+ X axis enable
                         //   ||||||+- Y axis enable
                         //   |||||+-- Z axis enable
                         //   ||||+--- PD: 0: power down, 1: active
                         //   ||++---- BW1-BW0: cut off 12.5[Hz]
                         //   ++------ DR1-DR0: ODR 95[HZ]

    escR.attach(ESC_PIN_R);
  escL.attach(ESC_PIN_L);
  
  Serial.println("Writing maximum output.");
  
  escR.writeMicroseconds(MAX_SIGNAL);
  escL.writeMicroseconds(MAX_SIGNAL);
  
  Serial.println("Wait 2 seconds.");
  delay(2000);
  Serial.println("Writing minimum output");
  
  escR.writeMicroseconds(MIN_SIGNAL);
  escL.writeMicroseconds(MIN_SIGNAL);
  
  Serial.println("Wait 2 seconds. Then motor starts");
  delay(2000);
  
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

  //６軸姿勢推定　MadgwickFilter.updateIMU(角速度x,角速度y,角速度z,加速度x,加速度y,加速度z);
  MadgwickFilter.updateIMU(gx-sumgx,gy-sumgy,gz-sumgz,ax,ay,az);

  //ラジアン取得
  float roll = MadgwickFilter.getRollRadians();
  float pitch = MadgwickFilter.getPitchRadians();
  float yaw = MadgwickFilter.getYawRadians();
  //誤差修正(20秒)
  if(error_modifying){
    em_time += 10;
    if(em_time > 20000/*20秒*/){
      em_roll = roll;
      em_pitch = pitch;
      em_yaw = yaw;
      error_modifying = false;}
   }
  Serial.print("e,");//データ識別用
  Serial.print(roll -em_roll);
  Serial.print(",");
  Serial.print(pitch - em_pitch);
  Serial.print(",");
  Serial.println(yaw - em_yaw);


        /*Serial.print(gx-sumgx);
        Serial.print(",");
        Serial.print(gy-sumgy);
        Serial.print(",");
        Serial.println(gz-sumgz); 
 
        Serial.print(ax);
        Serial.print(",");
        Serial.print(ay);
        Serial.print(",");
        Serial.println(az);*/

//コピペ


      //姿勢制御追加
      float Kp = 3;         // 比例ゲインKp
      float Ki = 3;         // 比例ゲインKi
      float Kd = 3;         // 比例ゲインKd
      float target = 0;     // 目標角度[rad]

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

        last_err = err;                                     // 現在の偏差を保存
        last_micros = current_micros;                       // 現在の時間を保存     
      Serial.println("test");
      Serial.println(em_time);             
  if(em_time > 20000 ){       
  switch(phase){
    case 1:
   {
      Serial.println("Mode-A実行中");

        escR.writeMicroseconds(1500);
        escL.writeMicroseconds(1500);    
        //右に流されたときの修正
        //if(roll<-0.5){
        if(roll-em_roll>0.5/*傾き（ラジアン表記）*/){
          //現在の時間を取得
          unsigned long current_micros = micros(); 
          while(micros()<current_micros + 1000000/*出力時間*/){
              //機体をyaw方向に+90度回転させる。
              escR.writeMicroseconds(1500/*入力電圧値*/);
          }
          while(micros()<current_micros + 1000000/*出力時間*/){
              //プロペラを左右回してroll方向の傾きを修正する。    
              escR.writeMicroseconds(1600/*入力電圧値*/);
              escL.writeMicroseconds(1200/*入力電圧値*/);
          }    
        }
          //左に流された時の修正
          //if(roll>-5.5){
          if(roll-em_roll<-0.5/*傾き（ラジアン表記）*/){
          //現在の時間を取得
          unsigned long current_micros = micros(); 
          while(micros()<current_micros + 1000000/*出力時間*/){
              //機体をyaw方向に+90度回転させる。
              escR.writeMicroseconds(1500/*入力電圧値*/);
          }
          while(micros()<current_micros + 1000000/*出力時間*/){
              //プロペラを左右回してroll方向の傾きを修正する。    
              escR.writeMicroseconds(1200/*入力電圧値*/);
              escL.writeMicroseconds(1600/*入力電圧値*/);
        }
        }
        //姿勢制御追加
        // PIDでモータを制御
        //test
       /* Serial.print(P);
        Serial.print(",");
        Serial.print(D);
        Serial.print(",");
        Serial.println(I);
        Serial.print(",");
        Serial.println(P+D+I);
        Serial.print(",");*/

        if(err > -0.30 || err < 0.30 )
            {
              phase = 2; //機体が安定した場合Mode-Bに移行する
              Serial.println("Mode-Bへ移行"); 
            }
        break;
   }
  case 2://Mode-B
  {
    
      escR.writeMicroseconds(1400);
      escL.writeMicroseconds(1400);
      Serial.println("mode_B実行中");
      
     CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial.print("CurrentDistance=");
          Serial.println(CurrentDistance);
     
        // Goalまでの偏角を計算する
       Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
       for (int i = 0; i < 15; i++){
            Vector norm = compass.readNormalize();
            heading = atan2(norm.YAxis, norm.XAxis);
         }
        Angle_gy270 = CalculateHeading(heading);
         // どちらに回ればいいか計算
              rrAngle = -Angle_gy270 + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_gy270 - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }
            if (rrAngle > llAngle){
              //反時計回り
              if (llAngle > 20){
                 RightRotating();//右側のモータの回転数を上げる
              }
            }else{
              //時計回り
              if (rrAngle > 20){
                LeftRotating();//左側のモータの回転数を上げる
              }
            }       
            if(err <= -0.30 || err >= 0.30)
            {
              phase = 1;//機体が安定していない場合Mode-Aに移行する
              Serial.println("Mode-Aへ移行"); 
            }
            if(desiredDistance < CurrentDistance){
              //phase = F;距離が理想値よりも小さくなったらMode-Fに移行する
              Serial.println("Mode-Fへ移行");
            }
        break;   
}
}       
}
    delay(10);
  //100Ｈｚに合わせる
}
