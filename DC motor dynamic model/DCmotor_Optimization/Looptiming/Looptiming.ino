void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t period = 0.16667 * 60000L;       // 5 minutes
Serial.println("====start========");
for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
   Serial.println("1. ");
}
Serial.println("--------------------------------------------------------------------");
}
