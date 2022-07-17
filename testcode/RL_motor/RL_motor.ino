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
  Serial.begin(9600);
  

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
}

void loop() {  
  for(int i = 100; i <= 2500; i = i + 50){ 
    volume = i;  //可変抵抗の値を1.0で掛けて変数volumeに入れる．値を調整したい場合は倍率を変更する．
    sprintf(message, "Pulse Width: %d micro sec", volume);  //シリアルモニタに表示するメッセージを作成
    Serial.println(message);  //可変抵抗の値をシリアルモニタに表示
    
    escR.writeMicroseconds(volume);
    escL.writeMicroseconds(volume);
    
    delay(500);
  }
}
