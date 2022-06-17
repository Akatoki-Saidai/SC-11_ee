//Aが右，Bが左を想定
const int A_PHASE = 3;//アナログピンを選択
const int A_ENABLE = 5;//アナログピンを選択
const int B_PHASE = 6;//アナログピンを選択
const int B_ENABLE = 9;//アナログピンを選択
int i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A_PHASE,OUTPUT);
  pinMode(A_ENABLE,OUTPUT);
  pinMode(B_PHASE,OUTPUT);
  pinMode(B_ENABLE,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
  delay(3000);
  stopping();
  delay(3000);
  back();
  delay(3000);
  turn_right();
  delay(3000);
  turn_left();
  delay(3000);
  accel();
  delay(1000);
  decel();
  delay(2000);
 }

//前進
int forward(){
 analogWrite(A_ENABLE,255);
 digitalWrite(A_PHASE,HIGH);
 analogWrite(B_ENABLE,255);
 digitalWrite(B_PHASE,HIGH);
}

//後退
int back(){
   analogWrite(A_ENABLE,255);
   digitalWrite(A_PHASE,LOW);
   analogWrite(B_ENABLE,255);
   digitalWrite(B_PHASE,LOW); 
}

//静止
int stopping(){
   analogWrite(A_ENABLE,0);
   digitalWrite(A_PHASE,LOW);
   analogWrite(B_ENABLE,0);
   digitalWrite(B_PHASE,LOW); 
}

//右旋回
int turn_right(){
   analogWrite(A_ENABLE,0);
   digitalWrite(A_PHASE,LOW);
   analogWrite(B_ENABLE,255);
   digitalWrite(B_PHASE,HIGH); 
}

//左旋回
int turn_left(){
   analogWrite(A_ENABLE,255);
   digitalWrite(A_PHASE,HIGH);
   analogWrite(B_ENABLE,0);
   digitalWrite(B_PHASE,LOW); 
}

//加速
int accel(){
  for(i=5;i==255;i=i+25){
    analogWrite(A_ENABLE,i);
    digitalWrite(A_PHASE,HIGH);
    analogWrite(B_ENABLE,i);
    digitalWrite(B_PHASE,HIGH);
    delay(500);
  }
  forward();
  delay(3000);
  stopping();
}

//減速
int decel(){
  for(i=255;i==5;i=i-25){
    analogWrite(A_ENABLE,i);
    digitalWrite(A_PHASE,HIGH);
    analogWrite(B_ENABLE,i);
    digitalWrite(B_PHASE,HIGH);
    delay(500); 
  }
  stopping();
}
