/*
PID Ball Balancer, 1 axis
Balances a beam with a ball on it at the midpoint of the track
Boston University ENG ME 360
Ben Jacobs, Al Levine, Marina Lyons
*/

#include <Servo.h>
//#include <PID_v1.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Pin defs
#define pin_dist_trig 2 // ultrasonic sensor trigger input pin
#define pin_dist_echo 3 // ultrasonic sensor trigger output pin
#define pin_serv 5 // servo control pin

// distance vars
double dist = 0; // current distance between ball and sensor
double dist_max = 61.5; // far end of track (cm)
double dist_min = 1.5; // near end of track (cm)

// servo vars
int ang_mid = 1500; // 1500 is in the middle -> +500 = 45deg CW, +250 = 22.5 deg CW
int ang_min = 1400; // 1000 is fully counter-clockwise 
int ang_max = 1600; // 2000 is fully clockwise
int set = ang_min; // variable for setting angle of servo
Servo fulcrum; // create servo

// PID
#define kp 0.8
#define ki 0.02
#define kd 0.75
double Stpt = (dist_max - dist_min)/2; // desired position (center of track = length of track/2)
double tErr, pErr = 0;
int out;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
  // Pin modes
  pinMode(pin_dist_trig, OUTPUT);
  pinMode(pin_dist_echo, INPUT);
  pinMode(pin_serv, OUTPUT);
  // Serial
  Serial.begin(9600); // start serial output for monitoring
  // Servo
  fulcrum.attach(pin_serv); // attach servo to pin
  fulcrum.writeMicroseconds(ang_mid); // set servo to midpoint
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  dist = readDist(pin_dist_echo, pin_dist_trig, tErr, pErr); //get distance reading
  out = PID(stpt, dist); //PID math
  //set = map(out, dist_min, dist_max, ang_min, ang_max); // map(value, fromLow, fromHigh, toLow, toHigh) , distance -> servo angle
  set = map(out, dist_min, dist_max, ang_min, ang_max); // reverse map, comment above
  
  Serial.print("New angle (ms): "); //print angle in microseconds
  Serial.println(set);
  
  delay(10);
  fulcrum.writeMicroseconds(set); // write new angle to servo
  delay(10);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

double readDist(int echo, int trig) {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long t = pulseIn(echo, HIGH);
  
  dist = duration * 0.034 / 2; // Speed of sound  divided by 2 (go and back)
  
  Serial.print("Distance (cm): "); // Prints the distance on the Serial Monitor
  Serial.println(dist);
  
  return dist;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int PID(double setpoint,double distance){
  double error = setpoint - distance;

  double Pval = error * kp;
  double Ival = toError * ki;
  double Dval = (error - priError) * kd;

  double PIDvalue = Pval + Ival + Dval;
  priError = error;
  toError += error;
  Serial.print("PID: ");
  Serial.println(PIDvalue);
  int Fvalue = (int)PIDvalue;
  
  if(Fvalue < dist_min){ Fvalue = dist_min;} //setting boundaries for distance
  if(Fvalue > dist_max){ Fvalue = dist_max;}
  
  return Fvalue
}