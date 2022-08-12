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
  Serial.begin(115200);
  while (!Serial.available());  //シリアルポートで何か入力されるまで待ちます
  Serial.read();

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

  for(int i = 900; i < 1600; i +=5){ 
    motor(i, i);
    delay(100);
    Serial.println(i);
  }
  motor(1600,1600);
  
}

void loop() {
}

void motor(int left, int right){
  escR.writeMicroseconds(right);
  escL.writeMicroseconds(left);
}
