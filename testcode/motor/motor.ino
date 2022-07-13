#include <Servo.h>

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define ESC_PIN 6  //ESCへの出力ピン
int volume;  //可変抵抗の値を入れる変数
char message[50];  //シリアルモニタへ表示する文字列を入れる変数

Servo esc;  //Servoオブジェクトを作成する．今回はESCにPWM信号を送るので，`esc`と命名している．

void setup() {
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  while (!Serial.available());  //シリアルポートで何か入力されるまで待ちます
  Serial.read();

  esc.attach(ESC_PIN);  //ESCへの出力ピンをアタッチします
  Serial.println("Writing maximum output.");
  esc.writeMicroseconds(MAX_SIGNAL);  //ESCへ最大のパルス幅を指示します
  Serial.println("Wait 2 seconds.");
  delay(2000);
  Serial.println("Writing minimum output");
  esc.writeMicroseconds(MIN_SIGNAL);  //ESCへ最小のパルス幅を指示します
  Serial.println("Wait 2 seconds. Then motor starts");
  delay(2000);
}

void loop() {  
  for(int i = 1000; i <= 2000; i = i + 100){ 
    volume = i;  //可変抵抗の値を1.0で掛けて変数volumeに入れる．値を調整したい場合は倍率を変更する．
    sprintf(message, "Pulse Width: %d micro sec", volume);  //シリアルモニタに表示するメッセージを作成
    Serial.println(message);  //可変抵抗の値をシリアルモニタに表示
    esc.writeMicroseconds(volume);  // パルス幅 `volume` のPWM信号を送信する
    delay(1000);
  }
}
