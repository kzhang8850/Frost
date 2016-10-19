int angle;
int power;

String input;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = "default";
  angle = 0;
  power = 10;
  
}

void loop() {
  
  if input != "default"){
    Serial.print("Current angle: ");
    Serial.println(angle);
    Serial.print("Current Power: ");
    Serial.println(power);
   }

}

void serialEvent(){
  input = Serial.readString();
}

