void setup() {
  // put your setup code here, to run once:
  pinMode(3,OUTPUT); //Motor right
  pinMode(4,OUTPUT); //Motor right
  pinMode(5,OUTPUT); //Speed Motor right
  pinMode(6,OUTPUT); //Motor left
  pinMode(7,OUTPUT); //Motor left
  pinMode(8,OUTPUT); //Speed Motor left

}

void loop() {
  // put your main code here, to run repeatedly:

  //forward
  forward(150);

  //right
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  analogWrite(5,200);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  analogWrite(8,200);
  delay(2500);

  //left
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
  analogWrite(5,200);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  analogWrite(8,200);
  delay(2500);
}

void forward(int speed)
{
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  analogWrite(5,speed);
  analogWrite(8,0);
  //delay(2500);
}
