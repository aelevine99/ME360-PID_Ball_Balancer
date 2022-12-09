/*
  PID Ball Balancer, 1 axis
  Balances a beam with a ball on it at the midpoint of the track
  Boston University ENG ME 360
  Ben Jacobs, Al Levine, Marina Lyons
*/

#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Pin defs
#define pin_trig 2  // ultrasonic sensor trigger input pin
#define pin_echo 3  // ultrasonic sensor trigger output pin
#define pin_serv 5  // servo control pin

// distance vars
double dist = 0;         // current distance between ball and sensor
double dist_max = 62.5;  // far end of track (cm)
double dist_min = 1.5;   // near end of track (cm)
Ultrasonic disto(pin_trig, pin_echo);

// servo vars
int ang_mid = 90;  // 1500 is in the middle -> +500 = 45deg CW, +250 = 22.5 deg CW
int range = 10;
int ang_min = ang_mid - range;                               // 1000 is fully counter-clockwise
int ang_max = ang_mid + range;                               // 2000 is fully clockwise

int set = ang_mid;                                // variable for setting angle of servo
Servo fulcrum;                                    // create servo

// PID
#define kp 0.9
#define ki 0.05
#define kd 0.4
double setpt = 30; //(dist_max-dist_min / 2) + dist_min;  // desired position (center of track = length of track/2)
double toErr, priErr, PIDvalue = 0;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
  // Serial
  Serial.begin(9600);  // start serial output for monitoring
  // Pin modes
  Serial.println("Initializing pins...");
  pinMode(pin_trig, OUTPUT);
  pinMode(pin_echo, INPUT);
  pinMode(pin_serv, OUTPUT);
  // Servo
  Serial.println("Initializing servo...");
  fulcrum.attach(pin_serv);            // attach servo to pin
  fulcrum.writeMicroseconds(ang_mid);  // set servo to level
  // Misc
  Serial.print("Setpoint: ");
  Serial.println(ang_mid);
  Serial.println("Wait 5 sec");
  delay(5000);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  //Get and print distance
  dist = disto.read(CM);  //get distance reading in cm
  if(dist>dist_max){dist=dist_max;}
  Serial.print("Distance (cm): ");
  Serial.println(dist);
  delay(10);
  // PID math
  PID(setpt, dist);
  // Angle
  set = map(PIDvalue, dist_min, dist_max, ang_min, ang_max);  // reverse map, comment above
  if (set < ang_min) { set = ang_min; }
  if (set > ang_max) { set = ang_max; }
  Serial.print("New angle (ms): ");  //print angle in microseconds
  Serial.println(set);
  delay(10);
  //Servo
  delay(1000);
  fulcrum.write(set);  // write new angle to servo
  
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void PID(double setpoint, double distance) {
  double error = distance - setpoint;

  double Pval = error * kp;
  double Ival = toErr * ki;
  double Dval = (error - priErr) * kd;

  PIDvalue = Pval + Ival + Dval;
  priErr = error;
  toErr += error;

  if (PIDvalue < dist_min) {
    PIDvalue = dist_min;  //setting boundaries for distance
  }
  if (PIDvalue > dist_max) {
    PIDvalue = dist_max;
  }

  Serial.print("PID: ");
  Serial.println(PIDvalue);
}