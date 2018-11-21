//functions
void getRSSI();
void calculateDistance();
uint8_t getDistanceFront();
void initialize();
void setMode(uint8_t mode);
long getLocation(long rssi) ;
void getNewRSSI(long slave, long master);

//Global variables
extern long RSSIValues[7];
extern long finalMasterRSSI;
extern unsigned long sensorDistance;
extern long finalSlaveRSSI;
extern uint8_t Mode;
extern long RSSI_Value;
extern uint8_t rssiTime;
extern uint16_t tempRssiTime;
extern int RSSIValuesWithDistance[7];

#define STARTBYTE 0xFF
#define PACKETSIZE 254
#define MINPACKET 8
#define TCP 0
#define ADHOC 0
#define FWD 0
#define ACK 1
#define ADDRESS 0
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

#define REDLED A11
#define WHITELED 44
#define YELLOWLED A12
#define ORANGELED 42


#define TIMER1COUNT 64286  //50Hz

//External commands, communicated with another robot (in Adhoc mode) or TCP
#define NOCOMMAND 0
#define MOVEFORWARD 0x01
#define MOVEFORWARDTIME 0x02
#define MOVEBACK 0x03
#define MOVEBACKTIME 0x04
#define TURNLEFT 0x05
#define TURNRIGHT 0x06
#define STOP 0x07
#define DISTANCEFRONT 0x0A
#define GETHEADING 0x0D 
#define GETID 0x0F

#define SETMODE 0x24
#define MODE1 0x25
#define MODE2 0x26
#define MODE3 0x27

#define MASTERRSSI 0x31

#define RECEIVEARRAY 0x45


//Internal commands, communicated with ESP32
#define INT_ID 0x01
#define INT_SSID_PWD 0x02
#define INT_MATRIX 0x03
#define INT_RSSI 0x04
#define INT_IP 0x05
#define INT_DEMO 0x06

#define NODECOUNT 16