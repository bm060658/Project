//motor right

int ENA =5; //analog pin5
int IN1 =6;
int IN2 =7;

//motor leaft

int ENB =3; //analog pin3
int IN3 =2;
int IN4 =4;

void setup() {
  
  pinMode(6,OUTPUT); //Motor right
  pinMode(7,OUTPUT); //Motor right
  pinMode(5,OUTPUT); //Speed Motor right
  pinMode(2,OUTPUT); //Motor left
  pinMode(4,OUTPUT); //Motor left
  pinMode(3,OUTPUT); //Speed Motor left
  

}

void loop() {
  

  //forward
  forward();
  //delay(2000);

  //right
  //right();
  //delay(2000);
  

  //left
  //left();
  //delay(2000);
  
}

void forward()
{
  
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  analogWrite(5,170);
  digitalWrite(2,LOW);
  digitalWrite(4,HIGH);
  analogWrite(3,170);
  
}
void right()
{
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  analogWrite(5,170);
  digitalWrite(2,HIGH);
  digitalWrite(4,LOW);
  analogWrite(3,170);
}
void left()
{
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  analogWrite(5,170);
  digitalWrite(2,LOW);
  digitalWrite(4,HIGH);
  analogWrite(3,170);
}


