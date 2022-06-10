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
  Serial.println(acX);
  Serial.print("y:");
  Serial.println(acY);
  Serial.print("z:");
  Serial.println(acZ);  
  delay(1000);
}
