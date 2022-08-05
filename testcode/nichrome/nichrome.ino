int sec = 3;
void setup() {
  // put your setup code here, to run once:
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW)
  while (!Serial.available());
  Serial.read();
  for(int i = 1; i <= 3; i++){
    Serial.print("Wait for ")
    Serial.print(i)
    Serial.print(" sec")
    Serial.println()
    delay(1000);
  }
  digitalWrite(6,HIGH);
  delay(sec*1000);
  digitalWrite(6,LOW);
  Serial.println("Motor released");
}

void loop() {
  // put your main code here, to run repeatedly:

}