int angle;
int power;
int counter;
char separator[] = ",e";
int inputs[2];
int i = 0;
char *token = "e";
String input;
String angle2;
String power2;
boolean received = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = "default";
  angle = 0;
  power = 10;
  
}

void loop() {
  
  if (received){
    Serial.print("Current Angle: ");
    Serial.println(angle);
    Serial.print("Current Power: ");
    Serial.println(power);
    received = false;
   }

}

void serialEvent(){
  if(Serial.available()){
    input = Serial.readString();
    Serial.print("captured String is : "); 
    Serial.println(input); //prints string to serial port out
    Serial.println(input);
    int len = input.length()+1;
    char realinput[len];
    input.toCharArray(realinput, len);
    Serial.println(realinput[1]);

    counter = input.indexOf(",");
    angle = input.substring(0,counter).toInt();
    power = input.substring(counter+1,len).toInt();

    received = true;
  }
}

