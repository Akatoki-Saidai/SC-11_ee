int xpin = A7;
int ypin = A6;
int zpin = A5;
int acX, acY, acZ;
double mX, mY, mZ;

void setup() {
  Serial.begin(115200);
}

void loop() {
  acX = analogRead(xpin);
  acY = analogRead(ypin);
  acZ = analogRead(zpin);
  mX = 3*(acX-511.5)/511.5;
  mY = 3*(acY-511.5)/511.5;
  mZ = 3*(acZ-511.5)/511.5;
  Serial.print("X:");
  Serial.println(9.8*(mX+0.92)/(-0.42));
  Serial.print("Y:");
  Serial.println(9.8*(mY+0.93)/(-0.41));
  Serial.print("Z:");
  Serial.println(9.8*(mZ+0.94)/(-0.42));
  delay(1000);
}
/*
acX,acY,acZは0~1023の値をとり、
加速度センサは±3gまで測定できることから、
出力値が0のとき‐３g、出力値が1023のとき3g、
出力値が511.5のとき0、と思われる。
*/
/*
任意の一軸の測定値をM,実際の値をTとし(M,Tの値は重力加速度との比とする)、誤差をEとすると、
E=M-T
誤差Eの大きさはTの値によって変わるので、EはTの関数として表せる。
EをTの一次までで表すと、
E=er1+er2*T     (er1,er2は定数)
二つの式から、
M=er1+(1+er2)*T
ここで、軸の正の向きを鉛直下向きにすると、T=1より、
M1=er1+(1+er2)
また、鉛直上向きでは、T=-1より、
M2=er1-(1+er2)
この二つの式から、er1,er2は、
er1=(M1+M2)/2
er2=-1+(M1-M2)/2

M=er1+(1+er2)*Tより、
T=(M-er1)/(1+er2)
これにより、誤差補正ができる。
*/
