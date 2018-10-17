#define MOTOR_FRONT_LEFT_P  35
#define MOTOR_FRONT_LEFT_N  33
#define MOTOR_BACK_LEFT_P  27
#define MOTOR_BACK_LEFT_N  25
#define MOTOR_FRONT_RIGHT_P 39
#define MOTOR_FRONT_RIGHT_N  37
#define MOTOR_BACK_RIGHT_P  31
#define MOTOR_BACK_RIGHT_N    29
#define ULTRASONIC_FRONT_TRIGGER   43
#define ULTRASONIC_FRONT_ECHO   41

//Custom variables
uint8_t prevFlag = 0;
uint8_t currentFlag = 1;
int8_t change = 0;
uint8_t prev = 0;
uint8_t current = 0;

void initGPIO()
{
 pinMode(MOTOR_FRONT_LEFT_P,OUTPUT);
 pinMode(MOTOR_FRONT_LEFT_N,OUTPUT);
 pinMode(MOTOR_BACK_LEFT_P,OUTPUT);
 pinMode(MOTOR_BACK_LEFT_N,OUTPUT);
 pinMode(MOTOR_FRONT_RIGHT_P,OUTPUT);
 pinMode(MOTOR_FRONT_RIGHT_N,OUTPUT);
 pinMode(MOTOR_BACK_RIGHT_P,OUTPUT);
 pinMode(MOTOR_BACK_RIGHT_N,OUTPUT);

 stopMotors();

 pinMode(ULTRASONIC_FRONT_TRIGGER, OUTPUT);
 pinMode(ULTRASONIC_FRONT_ECHO, INPUT);
}

//Get distance in cm from front ultrasonic sensor (In blocking mode)
uint8_t getDistanceFront()
{
 digitalWrite(ULTRASONIC_FRONT_TRIGGER, LOW); 
 delayMicroseconds(2); 

 digitalWrite(ULTRASONIC_FRONT_TRIGGER, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(ULTRASONIC_FRONT_TRIGGER, LOW);
 long duration = pulseIn(ULTRASONIC_FRONT_ECHO, HIGH);

//  Serial.println(duration);

 return duration/58.2;
}

void stopMotors()
{
  digitalWrite(MOTOR_FRONT_LEFT_P,LOW);
  digitalWrite(MOTOR_FRONT_LEFT_N,LOW);

  digitalWrite(MOTOR_BACK_LEFT_P,LOW);
  digitalWrite(MOTOR_BACK_LEFT_N,LOW);
  
  digitalWrite(MOTOR_FRONT_RIGHT_P,LOW);
  digitalWrite(MOTOR_FRONT_RIGHT_N,LOW);

  digitalWrite(MOTOR_BACK_RIGHT_P,LOW);
  digitalWrite(MOTOR_BACK_RIGHT_N,LOW);

  delay(200);  
}


void moveForward()

{
  stopMotors();
  digitalWrite(MOTOR_FRONT_LEFT_P,HIGH);
  digitalWrite(MOTOR_FRONT_LEFT_N,LOW);

  digitalWrite(MOTOR_BACK_LEFT_P,HIGH);
  digitalWrite(MOTOR_BACK_LEFT_N,LOW);
  
  digitalWrite(MOTOR_FRONT_RIGHT_P,HIGH);
  digitalWrite(MOTOR_FRONT_RIGHT_N,LOW);

  digitalWrite(MOTOR_BACK_RIGHT_P,HIGH);
  digitalWrite(MOTOR_BACK_RIGHT_N,LOW);

}

void moveBack()
{
  stopMotors();
  digitalWrite(MOTOR_FRONT_LEFT_P,LOW);
  digitalWrite(MOTOR_FRONT_LEFT_N,HIGH);

  digitalWrite(MOTOR_BACK_LEFT_P,LOW);
  digitalWrite(MOTOR_BACK_LEFT_N,HIGH);
  
  digitalWrite(MOTOR_FRONT_RIGHT_P,LOW);
  digitalWrite(MOTOR_FRONT_RIGHT_N,HIGH);

  digitalWrite(MOTOR_BACK_RIGHT_P,LOW);
  digitalWrite(MOTOR_BACK_RIGHT_N,HIGH);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing GPIO");
    initGPIO();
    delay(1000);
    Serial.println("Setup complete");
}

void loop() {
    delay(100);
    change = (int8_t)current - (int8_t)prev;
    Serial.print("Current ");
    Serial.println(current);
    Serial.print("previous ");
    Serial.println(prev);
    Serial.print("Difference ");
    Serial.println(change);
    prev = current;
    current = getDistanceFront();
    if(change < -1) {
        prevFlag = currentFlag;
        currentFlag = 2;
    } else if(change > 1) {
        prevFlag = currentFlag;
        currentFlag = 3;
    } else {
        prevFlag = currentFlag;
        currentFlag = 1;
    }

    if(currentFlag != prevFlag) {
        switch(currentFlag) {
            case 1:
                stopMotors();
                break;
            case 2:
                moveBack();
                break;
            case 3:
                moveForward();
                break;
        }
    }
}