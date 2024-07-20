#include <SoftwareSerial.h>   //header file of software serial port
#include <Servo.h>

///////////////////////Inputs/outputs///////////////////////
SoftwareSerial Serial1(2, 3); //define software serial port name as Serial1 and define pin2 as RX & pin3 as TX
Servo myservo;  // create servo object to control a servo, later attached to D9

////////////////////////Variables///////////////////////
int dist;                     //actual distance measurements of LiDAR
int strength;                 //signal strength of LiDAR
int check;                    //save check value
int i;
int uart[9];                   //save data measured by LiDAR
const int HEADER = 0x59;      //frame header of data package
int period = 50;  //Refresh rate period of the loop is 50ms

float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;

///////////////////PID constants///////////////////////
float kp = 20;//
float ki = 0.2;
float kd = 3500;
float distance_setpoint = 30;         //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;

float get_TFminiPlus_distance() {
  while (Serial1.available()) { //check if serial port has data input
    if (Serial1.read() == HEADER) { //assess data package frame header 0x59
      uart[0] = HEADER;
      if (Serial1.read() == HEADER) { //assess data package frame header 0x59
        uart[1] = HEADER;
        for (i = 2; i < 9; i++) { //save data in array
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
          dist = uart[2] + uart[3] * 256; //calculate distance value
          return dist; //return measured distance value of LiDAR
        }
      }
    }
  }
  return distance_setpoint; //return setpoint if no valid data
}

void setup() {
  Serial.begin(9600);         //set bit rate of serial port connecting Arduino with computer
  Serial1.begin(115200);      //set bit rate of serial port connecting LiDAR with Arduino

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(85);

  time = millis();
}

void loop() {
  if (millis() > time + period) {
    time = millis();
    distance = get_TFminiPlus_distance(); // Read distance from TFmini Plus
    distance_error = distance_setpoint - distance;
    PID_p = kp * distance_error;
    float dist_difference = distance_error - distance_previous_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);

    if (-3 < distance_error && distance_error < 3) {
      PID_i = PID_i + (ki * distance_error);
    } else {
      PID_i = 0;
    }

    PID_total = PID_p + PID_i + PID_d;
    if (PID_total < -30) {
      PID_total = -30;
    }
    //    Serial.print(PID_total);
    Serial.print(PID_total);
    Serial.print("\t");
    PID_total = map(PID_total, -500, 500, 0, 150);


    //    if (PID_total < 20 && PID_total > 120) {
    //      PID_total = 0;
    //    }
    //    if (PID_total > 120) {
    //      PID_total = 160;
    //    }
    Serial.print(PID_total);
    myservo.write(PID_total);

    distance_previous_error = distance_error;

    // Print the distance and servo angle
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Servo Angle: ");
    Serial.println(PID_total + 30);
  }
}
