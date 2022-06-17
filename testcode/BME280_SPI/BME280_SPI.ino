#include <SPI.h>

//アドレス指定
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define CTRL_HUM 0xF2

//気温補正データ
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
 
//湿度補正データ
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
 
//気圧補正データ
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
int32_t adc_P, adc_T, adc_H;

int32_t  temp_cal;
uint32_t humi_cal, pres_cal;
double temp, humi, pres ,alt;
double p0=1013.25;//海面気圧（hPa）

void setup() {
  //シリアル通信初期化
  Serial.begin(9600);//シリアル通信を9600bpsで初期化

  //SPI初期化
  SPI.begin();//SPIを初期化、SCK、MOSI、SSの各ピンの動作は出力、SCK、MOSIはLOW、SSはHIGH
  SPI.setDataMode(SPI_MODE0);//SPIモードを「0」に設定・CPOL(クロック位相)=0,CPHA(クロック極性)=0
  SPI.setBitOrder(MSBFIRST);//SPI送受信用のビットオーダーを「MSBFIRST」に設定

  //BME280動作設定
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(CONFIG & 0x7F);//動作設定
  SPI.transfer(0x00);//「単発測定」、「フィルタなし」、「SPI 4線式」
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定

  //BME280測定条件設定
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(CTRL_MEAS & 0x7F);//測定条件設定
  SPI.transfer(0x24);//「温度・気圧オーバーサンプリングx1」、「スリープモード」
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定

  //BME280温度測定条件設定
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(CTRL_HUM & 0x7F);//湿度測定条件設定
  SPI.transfer(0x01);//「湿度オーバーサンプリングx1」
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定

  //BME280補正データ取得
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(0x88 | 0x80);//出力データバイトを「補正データ」のアドレスに指定、書き込みフラグを立てる
  for (i=0; i<26; i++){
    dac[i] = SPI.transfer(0x00);//dacにSPIデバイス「BME280」のデータ読み込み
  }
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定

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

  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(0xE1 | 0x80);//出力データバイトを「補正データ」のアドレスに指定、書き込みフラグを立てる
  for (i=0; i<7; i++){
    dac[i] = SPI.transfer(0x00);//dacにSPIデバイス「BM3280」のデータ読み込み
  }
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定

  dig_H2 = ((int16_t)((dac[1] << 8) | dac[0]));
  dig_H3 = ((uint8_t)(dac[2]));
  dig_H4 = ((int16_t)((dac[3] << 4) + (dac[4] & 0x0F)));
  dig_H5 = ((int16_t)((dac[5] << 4) + ((dac[4] >> 4) & 0x0F)));
  dig_H6 = ((int8_t)dac[6]);
  
  delay(1000);//1000msec待機(1秒待機)
}
 
void loop() {

  //BME280測定条件設定(1回測定後、スリープモード)
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(CTRL_MEAS & 0x7F);//測定条件設定
  SPI.transfer(0x25);//「温度・気圧オーバーサンプリングx1」、「1回測定後、スリープモード」
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定
  delay(10);//10msec待機

  //測定データ取得
  digitalWrite(SS, LOW);//SSピンの出力をLOW(0V)に設定
  SPI.transfer(0xF7 | 0x80);//出力データバイトを「気圧データ」のアドレスに指定、書き込みフラグを立てる
  for (i=0; i<8; i++){
    dac[i] = SPI.transfer(0x00);//dacにSPIデバイス「BME280」のデータ読み込み
  }
  digitalWrite(SS, HIGH);//SSピンの出力をHIGH(5V)に設定
  
  adc_P = ((uint32_t)dac[0] << 12) | ((uint32_t)dac[1] << 4) | ((dac[2] >> 4) & 0x0F);
  adc_T = ((uint32_t)dac[3] << 12) | ((uint32_t)dac[4] << 4) | ((dac[5] >> 4) & 0x0F);
  adc_H = ((uint32_t)dac[6] << 8) | ((uint32_t)dac[7]);
  
  pres_cal = BME280_compensate_P_int32(adc_P);//気圧データ補正計算
  temp_cal = BME280_compensate_T_int32(adc_T);//温度データ補正計算
  humi_cal = bme280_compensate_H_int32(adc_H);//湿度データ補正計算

  pres = (float)pres_cal / 100.0;//気圧データを実際の値に計算
  temp = (float)temp_cal / 100.0;//温度データを実際の値に計算
  humi = (float)humi_cal / 1024.0;//湿度データを実際の値に計算
  alt  = ((pow(p0/pres,-5.257)-1)*(temp + 273.15))/0.0065;

  //シリアルモニタ送信
  Serial.println("-----------------");
  Serial.print("Pressure:");//文字列「Pressure:」をシリアルモニタに送信
  Serial.print(pres);//「pres」をシリアルモニタに送信
  Serial.println("hPa ");//文字列「hPa 」をシリアルモニタに送信
  Serial.print("Temp:");//文字列「Temp:」をシリアルモニタに送信
  Serial.print(temp);//「temp」をシリアルモニタに送信
  Serial.println("°C ");//文字列「°C 」をシリアルモニタに送信
  Serial.print("Humidity:");//文字列「Humidity:」をシリアルモニタに送信
  Serial.print(humi);//「humi」をシリアルモニタに送信
  Serial.println("%");//文字列「%」をシリアルモニタに送信、改行
  Serial.print("Altitude：");
  Serial.print(alt);
  Serial.println("m");
  
  delay(1000);//1000msec待機(1秒待機)
}

//温度補正 関数
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T  = (t_fine * 5 + 128) >> 8;
  return T;
}

//湿度補正 関数
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
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
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}
