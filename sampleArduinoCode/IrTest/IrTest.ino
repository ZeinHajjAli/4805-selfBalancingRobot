
int ir = 8;

void setup() {
  
  pinMode(ir, INPUT);
  Serial.begin(9600);
  Serial.println("Serial Monitor Running");

}

void loop() {

  Serial.println(digitalRead(ir));

}
