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
  Serial.println(9.8*(mX+0.92)/(0.7*(1-0.42)));
  Serial.print("Y:");
  Serial.println(9.8*(mY+0.93)/(0.7*(1-0.41)));
  Serial.print("Z:");
  Serial.println(9.8*(mZ+0.94)/(0.7*(1-0.42)));

  delay(1000);
}
/*
acX,acY,acZは0~1023の値をとり、
加速度センサは±3gまで測定できることから、
出力値が0のとき‐３g、出力値が1023のとき3g、
出力値が511.5のとき0、と思われる。
*/
