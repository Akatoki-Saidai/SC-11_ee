#include <Servo.h>

<<<<<<< Updated upstream
#define MAX_SIGNAL 1600  //PWM信号における最大のパルス幅[マイクロ秒]
=======
#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
  delay(4000);
  Serial.println("Writing minimum output");
  esc.writeMicroseconds(MIN_SIGNAL);  //ESCへ最小のパルス幅を指示します
  Serial.println("Wait 2 seconds. Then motor starts");
  delay(4000);
=======
  delay(3000);
  Serial.println("Writing minimum output");
  esc.writeMicroseconds(MIN_SIGNAL);  //ESCへ最小のパルス幅を指示します
  Serial.println("Wait 2 seconds. Then motor starts");
  delay(3000);
>>>>>>> Stashed changes
  Serial.println("Start 1000 PWM");
    // パルス幅 `volume` のPWM信号を送信する
}

void loop() {  
  esc.writeMicroseconds(1200);
  delay(1000);
}
