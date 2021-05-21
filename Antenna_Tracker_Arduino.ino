#include <PID_v1.h>
#include <Servo.h>

#define MotEnable 5        //Motor Enamble pin Runs on PWM signal
#define MotFwd 6          // Motor Forward pin
#define MotRev 7          // Motor Reverse pin

long count = 1;                //a counter
float alt1 = 1.45;            //altitude 1 (antenna)
float alt2, longitude2, latitude2, finalAngle;
float oldAngle = 0.0; 

const float LATITUDE1 = 39.099912;
const float LONGITUDE1 = -94.581213;

int servoPin = 10;
float servoPos = 0;

Servo myServo;

int User_Input = 0;     // This while convert input string into integer
int encoderPin1 = 2;     //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3;       //Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncoded = 0;       // Here updated value of encoder store.
volatile long encoderValue = 0;     // Raw encoder value

int PPR = 1600;            // Encoder Pulse per revolution.
int angle = 360;           // Maximum degree of motion.
int REV = 0;              // Set point REQUIRED ENCODER VALUE
int lastMSB = 0;
int lastLSB = 0;

double kp = 5 , ki = 1 , kd = 0.01;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(MotEnable, OUTPUT);
  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(115200); //initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH);            //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);           //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  myServo.attach(servoPin);
  
  TCCR1B = TCCR1B & 0b11111000 | 1;          // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);                 //set PID in Auto mode
  myPID.SetSampleTime(1);                   // refresh rate of PID controller
  myPID.SetOutputLimits(-125, 125);         // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
}

void loop() {
  while (Serial.available() == 0)         //program waits for serial input using an infinite loop
  {
  }
  String coord1 = Serial.readString();    //puts the received string in the latitude variable (drone latitude)
  latitude2 = coord1.toFloat();           //changes latitude from string to float
  
  while (Serial.available() == 0)          //program waits for serial input using an infinite loop
  {
  }
  String coord2 = Serial.readString();      //puts the received string in longitude variable (drone longitude)
  longitude2 = coord2.toFloat();            //changes longitude from string to float
  
  while (Serial.available() == 0)          //program waits for serial input using an infinite loop
  {
  }
  String coord3 = Serial.readString();      //puts the received string in the altitude variable (drone altitude)
  alt2 = coord3.toFloat();                  //changes altitude from string to float

  float dAlt = (alt2 - alt1) / 1000;                        //differnece in altitude
  Serial.print(count);                             //prints counter
  Serial.print(" Distance = ");
    
  float x = HaverSine(LATITUDE1, LONGITUDE1, latitude2, longitude2);  //calculates distance by calling haversine function
  Serial.print(x, 3);                              //prints distance with 3 decimal places
    
  count++;                                         //increases counter
  Serial.print(", altitude = ");
  Serial.print(dAlt);                              //prints difference in altitude
    
  float z = dAlt / x;                              //calculates ratio of altitude and distance
  float angle = atan(z) * 180 / PI;                //finds angle in radians and is converted to degrees
  Serial.print(" and the angle = ");
  Serial.print(angle, 3);                          //prints angle with 3 decimal places
    
  float angle2 = bearing(LATITUDE1, LONGITUDE1, latitude2, longitude2);     //calculates bearing angle by calling bearing function
  Serial.print(", and 2 angle = ");
  Serial.println(angle2, 6);                      //prints angle

  finalAngle = angle2 - oldAngle;
  oldAngle = angle2;
  
  servoPos = (int) angle;
  myServo.write(servoPos);

  User_Input = (int) finalAngle;

  if (User_Input < 0)
  {
    REV = map (User_Input, -360, 0, 0, 1600);
  }
  else 
  {
    REV = map (User_Input, 0, 360, 0, 1600); // mapping degree into pulse
  }
  //Serial.print("this is REV - "); 
  //Serial.println(REV);               // printing REV value  

  setpoint = REV;                    //PID while work to achive this value consider as SET value
  input = encoderValue ;           // data from encoder consider as a Process value
  //Serial.print("encoderValue - ");
  //Serial.println(encoderValue);
  myPID.Compute();                 // calculate new output
  pwmOut(output);
}

float HaverSine(float lat1, float lon1, float lat2, float lon2)
{
  float ToRad = PI / 180.0;
  float R = 6371;

  float dLat = (lat2 - lat1) * ToRad;
  float dLon = (lon2 - lon1) * ToRad;
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * ToRad) * cos(lat2 * ToRad) * sin(dLon/2) * sin(dLon/2);

  float c = 2 * atan2(sqrt(a), sqrt(1-a));

  float d = R * c;
  return d;
}

float bearing(float latit1,float longit1,float latit2,float longit2)
{
  float toRad = PI / 180.0;
  
  float latitud1 = latit1 * toRad;
  float latitud2 = latit2 * toRad;
  
  float dLat = (latit2 - latit1) * toRad;
  float dLon = (longit2 - longit1) * toRad;

  float y = sin(dLon) * cos(latitude2);
  float x = cos(latitud1)*sin(latitud2) - sin(latitud1)*cos(latitud2)*cos(dLon);
  float brng = atan2(y,x);
  
  brng = brng / toRad;// radians to degrees
  return brng;

}

void pwmOut(int out) {                               
  if (out > 0) {                         // if REV > encoderValue motor move in forward direction.    
    analogWrite(MotEnable, out);         // Enabling motor enable pin to reach the desire angle
    forward();                           // calling motor to move forward
  }
  else {
    analogWrite(MotEnable, abs(out));          // if REV < encoderValue motor move in forward direction.                      
    reverse();                            // calling motor to move reverse
  }
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time

}

void forward () {
  digitalWrite(MotFwd, HIGH); 
 digitalWrite(MotRev, LOW); 
  
}

void reverse () {
  digitalWrite(MotFwd, LOW); 
 digitalWrite(MotRev, HIGH); 
  
}

void finish () {
  digitalWrite(MotFwd, LOW); 
 digitalWrite(MotRev, LOW); 
  
}
