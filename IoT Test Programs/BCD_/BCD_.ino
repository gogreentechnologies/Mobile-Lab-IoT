
void setup() {
  Serial.begin(115200);  // serial monitor at 115200 bps
  // put your setup code here, to run once:
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(33, OUTPUT);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //0
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(33, LOW);
  delay(2000);

  //1
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(33, LOW);
  delay(2000);

  //2
  digitalWrite(25, LOW);
  digitalWrite(26, HIGH);
  digitalWrite(27, LOW);
  digitalWrite(33, LOW);
  delay(2000);

  //3
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(27, LOW);
  digitalWrite(33, LOW);
  delay(2000);

  //4
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, HIGH);
  digitalWrite(33, LOW);
  delay(2000);

  //5
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  digitalWrite(27, HIGH);
  digitalWrite(33, LOW);
  delay(2000);

  //6
  digitalWrite(25, LOW);
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
  digitalWrite(33, LOW);
  delay(2000);

  //7
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
  digitalWrite(33, LOW);
  delay(2000);

  //8
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(33, HIGH);
  delay(2000);

  //9
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(33, HIGH);
  delay(2000);
}
