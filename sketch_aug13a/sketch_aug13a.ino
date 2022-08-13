#include <SPI.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include<MadgwickAHRS.h>
#include <Servo.h>

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define ESC_PIN_R 4  //ESCへの出力ピン
#define ESC_PIN_L 5
#define rad2deg(a) ((a) / M_PI * 180.0) /* rad を deg に換算するマクロ関数 */
#define deg2rad(a) ((a) / 180.0 * M_PI) /* deg を rad に換算するマクロ関数 */
Servo escR;
Servo escL;

Madgwick MadgwickFilter;
DFRobot_QMC5883 compass;
const int BMP280_CS = 9;
const int L3GD20_CS = 10;
const int SD_CS = 4;
int xpin = A2;
int ypin = A1;
int zpin = A0;
int acX, acY, acZ;
float mX, mY, mZ;
float sumacc;
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define CTRL_HUM 0xF2
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;
unsigned char dac[26];
unsigned int i;
int32_t t_fine;
int32_t adc_P, adc_T;
int32_t  temp_cal;
uint32_t pres_cal;
float temp, pres ,alt;
float p0=1013.25;
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
short gyroX, gyroY, gyroZ;
float gyrox, gyroy, gyroz;
float magmagX, magmagY, magmagZ;
float heading, headingDegrees;

//直前に入力する目標値
float Goal_lat=35.860545000;
float Goal_lng=139.606940001;

//for GPS
float delta_lng;
float distance;
float azimuth;
float gps_latitude,gps_longtitude;

int phase = 1;
int phase_state = 0;
float boarderheight;
long starttime;
float roll, pitch, yaw;
void L3GD20_write(byte reg, byte val){
    digitalWrite(L3GD20_CS, LOW);
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(L3GD20_CS, HIGH);
}
byte L3GD20_read(byte reg){
    byte ret = 0;
    digitalWrite(L3GD20_CS, LOW);
    SPI.transfer(reg | L3GD20_RW);
    ret = SPI.transfer(0);
    digitalWrite(L3GD20_CS, HIGH);
    return ret;
}
void setup(){
    Serial.begin(115200);
    if(!compass.begin()){};
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    pinMode(BMP280_CS, OUTPUT);
    pinMode(L3GD20_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(BMP280_CS,HIGH);
    digitalWrite(L3GD20_CS,HIGH);
    digitalWrite(SD_CS,HIGH);
    digitalWrite(BMP280_CS, LOW);
    SPI.transfer(CONFIG & 0x7F);
    SPI.transfer(0x00);
    digitalWrite(BMP280_CS, HIGH);
    digitalWrite(BMP280_CS, LOW);
    SPI.transfer(CTRL_MEAS & 0x7F);
    SPI.transfer(0x24);
    digitalWrite(BMP280_CS, HIGH);
    digitalWrite(BMP280_CS, LOW);
    SPI.transfer(CTRL_HUM & 0x7F);
    SPI.transfer(0x01);
    digitalWrite(BMP280_CS, HIGH);
    digitalWrite(BMP280_CS, LOW);
    SPI.transfer(0x88 | 0x80);
    for (i=0; i<26; i++){
        dac[i] = SPI.transfer(0x00);}
    digitalWrite(BMP280_CS, HIGH);
    dig_T1 = ((uint16_t)((dac[1] << 8) | dac[0]));
    dig_T2 = ((int16_t)((dac[3] << 8) | dac[2]));
    dig_T3 = ((int16_t)((dac[5] << 8) | dac[4]));
    dig_P1 = ((uint16_t)((dac[7] << 8) | dac[6]));
    dig_P2 = ((int16_t)((dac[9] << 8) | dac[8]));
    dig_P3 = ((int16_t)((dac[11] << 8) | dac[10]));
    dig_P4 = ((int16_t)((dac[13] << 8) | dac[12]));
    dig_P5 = ((int16_t)((dac[15] << 8) | dac[14]));
    dig_P6 = ((int16_t)((dac[17] << 8) | dac[16]));
    dig_P7 = ((int16_t)((dac[19] << 8) | dac[18]));
    dig_P8 = ((int16_t)((dac[21] << 8) | dac[20]));
    dig_P9 = ((int16_t)((dac[23] << 8) | dac[22]));
    dig_H1 = ((uint8_t)(dac[25]));
    digitalWrite(BMP280_CS, LOW);
    SPI.transfer(0xE1 | 0x80);
    for (i=0; i<7; i++){
        dac[i] = SPI.transfer(0x00);    
    digitalWrite(BMP280_CS, HIGH);
    dig_H2 = ((int16_t)((dac[1] << 8) | dac[0]));
    dig_H3 = ((uint8_t)(dac[2]));
    dig_H4 = ((int16_t)((dac[3] << 4) + (dac[4] & 0x0F)));
    dig_H5 = ((int16_t)((dac[5] << 4) + ((dac[4] >> 4) & 0x0F)));
    dig_H6 = ((int8_t)dac[6]);
    delay(1000);}
    L3GD20_write(L3GD20_CTRL1, B00001111);
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
    escR.attach(ESC_PIN_R);
    escL.attach(ESC_PIN_L);
    
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(A3,OUTPUT);
    
    LEDsetting(7);
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
    for(int i=900; i <1400; i+=100){
      motor(i,i);
      delay(100);
     }
     motor(0,0);
    pinMode(6,OUTPUT);
    digitalWrite(6,LOW);
    LEDsetting(1);
    delay(1000);
}
void loop(){
    getDataBMP();
    getL3GD20();
    getAcc();
    getmagmag();
    //getmagmag2();GPSが使える場合
    sumacc = sqrt(mX*mX + mY*mY + mZ*mZ) -0.9 ;
    Serial.println(alt);
    Serial.print(alt);
    Serial.print("\t");
    Serial.println(sumacc);

switch(phase){
      case 1:{
        if(phase_state != 1){
          Serial.println("Mode-N: Move completed");
          boarderheight = alt;
          starttime = millis();
          phase_state = 1;
          LEDsetting(0);
          delay(1000);
          LEDsetting(2);
          }
        
        if((alt - boarderheight > 30) && (millis() - starttime > 180*1000) && (sumacc > 1.0)){
            Serial.println("Mode-N: Detected a fall");
            phase = 2;
            LEDsetting(3);
            digitalWrite(6,HIGH);
            delay(6*1000);
            digitalWrite(6,LOW);
          }
          break;
        }
        case 2:{
          if(phase_state != 2){
            Serial.println("Mode-A: Moved completed");
            phase_state = 2;
            LEDsetting(4);
            }
          if(gyroz > 90){
            motor(0,zaxis_control(gyroz));
           }else if(gyroz<-90){
            motor(zaxis_control(gyroz),0);
           }        
          if (abs(gyroz) < 90){
            phase = 3;
          }
            
          break;
          }
        case 3:{
          if(phase_state != 3){
            Serial.println("Mode-B: Moved completed");
            phase_state = 3;
            LEDsetting(5);
            }
            motor(1300,1300);
          //GPS使えない場合,当日ゴール方向の地磁気のx,y値を読み取る
          //右のモータ出力上げる
            if(headingDegrees>20 && headingDegrees<90){
              motor(1300,1100);
              //左のモーターの出力を上げる
            }else if(headingDegrees>270 && headingDegrees<345){
              motor(1100,1300);
            }

            //GPSが使える場合
            //右のモータの出力を上げる
            if(Angle_Goal-headingDegrees>20 && Angle_Goal-headingDegrees<90){
              motor(1300,1100);
              //左のモーターの出力を上げる
            }else if(Angle_Goal-headingDegrees>270 && Angle_Goal-headingDegrees<345){
              motor(1100,1300);
            }
            if(abs(gyroz)>90){
              phase = 2;
            }
            break;
        }
      }
}
int32_t BME280_compensate_T_int32(int32_t adc_T){
    int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T  = (t_fine * 5 + 128) >> 8;
    return T;
}
//湿度補正 関数
uint32_t bme280_compensate_H_int32(int32_t adc_H){
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800)); 
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
    ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}
//気圧補正 関数
uint32_t BME280_compensate_P_int32(int32_t adc_P){
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0){
    return 0; // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000){
    p = (p << 1) / ((uint32_t)var1);
    }else{
    p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}
void getDataBMP(){
    //BME280測定条件設定(1回測定後、スリープモード)
    digitalWrite(BMP280_CS, LOW);//BMP280_CSピンの出力をLOW(0V)に設定
    SPI.transfer(CTRL_MEAS & 0x7F);//測定条件設定
    SPI.transfer(0x25);//「温度・気圧オーバーサンプリングx1」、「1回測定後、スリープモード」
    digitalWrite(BMP280_CS, HIGH);//BMP280_CSピンの出力をHIGH(5V)に設定
//    delay(10);//10msec待機
    //測定データ取得
    digitalWrite(BMP280_CS, LOW);//BMP280_CSピンの出力をLOW(0V)に設定
    SPI.transfer(0xF7 | 0x80);//出力データバイトを「気圧データ」のアドレスに指定、書き込みフラグを立てる
    for (i=0; i<8; i++){
    dac[i] = SPI.transfer(0x00);//dacにSPIデバイス「BME280」のデータ読み込み
    }
    digitalWrite(BMP280_CS, HIGH);//BMP280_CSピンの出力をHIGH(5V)に設定
    adc_P = ((uint32_t)dac[0] << 12) | ((uint32_t)dac[1] << 4) | ((dac[2] >> 4) & 0x0F);
    adc_T = ((uint32_t)dac[3] << 12) | ((uint32_t)dac[4] << 4) | ((dac[5] >> 4) & 0x0F);
    pres_cal = BME280_compensate_P_int32(adc_P);//気圧データ補正計算
    temp_cal = BME280_compensate_T_int32(adc_T);//温度データ補正計算
    pres = (float)pres_cal / 100.0;//気圧データを実際の値に計算
    temp = (float)temp_cal / 100.0;//温度データを実際の値に計算
    alt  = ((pow(p0/pres,1/5.257) - 1)*(temp + 273.15))/0.0065;
}
void getL3GD20(){
    gyroX = L3GD20_read(L3GD20_X_H);
    gyrox = gyroX = (gyroX << 8) | L3GD20_read(L3GD20_X_L);
    gyroY = L3GD20_read(L3GD20_Y_H);
    gyroy = gyroY = (gyroY << 8) | L3GD20_read(L3GD20_Y_L);
    gyroZ = L3GD20_read(L3GD20_Z_H);
    gyroz = gyroZ = (gyroZ << 8) | L3GD20_read(L3GD20_Z_L);
    
    gyrox *= 0.00875; // +-250dps
    //x *= 0.0175;// +-500dps
    //x *= 0.07;  // +-2000dps
    gyroy *= 0.00875; // +-250dps
    gyroz *= 0.00875; // +-250dps
//    delay(10);
}
void getAcc(){
  acX = analogRead(xpin);
  acY = analogRead(ypin);
  acZ = analogRead(zpin);
  //電圧値を加速度(単位は重力加速度)に変換
  mX = 3*(acX-511.5)/511.5;
  mY = 3*(acY-511.5)/511.5;
  mZ = 3*(acZ-511.5)/511.5;
  //誤差補正
  mX = (mX+0.92)/(-0.42);
  mY = (mY+0.93)/(-0.41);
  mZ = -(mZ+0.94)/(-0.42);
}
float getmagmag(){
  Vector norm = compass.readNormalize();
  // Calculate heading
  heading = atan2(norm.YAxis, norm.XAxis);
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = -(9.0 + (2.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  headingDegrees = heading * 180/M_PI;
}

float getmagmag2(){
  Vector norm = compass.readNormalize();
  // Calculate heading
  heading = atan2(norm.YAxis, norm.XAxis);
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = -(9.0 + (2.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  //headingDegrees = heading * 180/M_PI;



  //Goalまでの距離を計算
  CurrentDistance=CalculateDis(GOAL_lng,GOAL_lat,gps_longtitude,gps_latitude);

  //Goalまでの偏角を計算
  Angle_Goal=CalculateAngle(GOAL_lng,GOAL_lat,gps_longitude,gps_latitude);

  
}

//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137; //

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

void LEDsetting(int num){
  switch (num){
  case 0:{
    digitalWrite(8,LOW);
    digitalWrite(7,LOW);
    digitalWrite(A3,LOW);
    break;
  }
  case 1:{
    digitalWrite(8,HIGH);
    digitalWrite(7,LOW);
    digitalWrite(A3,LOW);
    break;
  }
  case 2:{
    digitalWrite(8,LOW);
    digitalWrite(7,HIGH);
    digitalWrite(A3,LOW);
    break;
  }
  case 3:{
    digitalWrite(8,LOW);
    digitalWrite(7,LOW);
    digitalWrite(A3,HIGH);
    break;
  }
  
  case 4:{
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(A3,LOW);
    break;
  }
  case 5:{
    digitalWrite(8,HIGH);
    digitalWrite(7,LOW);
    digitalWrite(A3,HIGH);
    break;
  }
  case 6:{
    digitalWrite(8,LOW);
    digitalWrite(7,HIGH);
    digitalWrite(A3,HIGH);
    break;
  }
  case 7:{
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(A3,HIGH);
    break;
  }
}
}
void motor(int right, int left){
  escR.writeMicroseconds(right);
  escL.writeMicroseconds(left);
}
int zaxis_control(float x){
  if ((x > 90) && (x<190)){
   return (int) 3*x - 170+1000;
   }else if(x <= 190){
   return 395+1000;
   }
}
