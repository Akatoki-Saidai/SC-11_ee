int xpin = A7;
int ypin = A6;
int zpin = A5;
int acX, acY, acZ;

void setup() {
  Serial.begin(115200);
}

void loop() {
  acX = analogRead(xpin);
  acY = analogRead(ypin);
  acZ = analogRead(zpin);
  Serial.print("X:");
  Serial.println(29.4*(acX-511.5)/511.5);
  Serial.print("y:");
  Serial.println(29.4*(acY-511.5)/511.5);
  Serial.print("z:");
  Serial.println(29.4*(acZ-511.5)/511.5);
  delay(1000);
}
/*
acX,acY,acZは0~1023の値をとり、
加速度センサは±3gまで測定できることから、
出力値が0のとき‐３g、出力値が1023のとき3g、
出力値が511.5のとき0、と思われる。
*/
