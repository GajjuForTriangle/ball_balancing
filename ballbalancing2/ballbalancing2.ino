#include <SoftwareSerial.h>
#include <Servo.h>

// Define software serial port and pins for RX and TX
SoftwareSerial Serial1(2, 3); // RX, TX
Servo myservo; // Create a servo object

// Variables
int dist; // Distance measurements
int strength; // Signal strength (not used in this code)
int check; // Check value for data integrity
int i; // Loop counter
int uart[9]; // Array to store LiDAR data
const int HEADER = 0x59; // Frame header for LiDAR data
int period = 50; // Refresh rate period in milliseconds

float distance = 0.0;
float elapsedTime, time, timePrev; // Time control variables
float distance_previous_error, distance_error; // PID error variables

// PID constants
float kp = 20;
float ki = 0.5;
float kd = 5200;
float distance_setpoint = 30; // Desired distance in cm
float PID_p, PID_i, PID_d, PID_total;

float get_TFminiPlus_distance() {
  while (Serial1.available()) { // Check if serial port has data input
    if (Serial1.read() == HEADER) { // Look for frame header
      uart[0] = HEADER;
      if (Serial1.read() == HEADER) { // Confirm second header byte
        uart[1] = HEADER;
        for (i = 2; i < 9; i++) { // Read remaining data into array
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) { // Verify checksum
          dist = uart[2] + uart[3] * 256; // Calculate distance
          return dist; // Return distance value
        }
      }
    }
  }
  return distance_setpoint; // Return setpoint if no valid data
}

void setup() {
  Serial.begin(9600); // Set bit rate for serial communication with PC
  Serial1.begin(115200); // Set bit rate for serial communication with LiDAR

  myservo.attach(9); // Attach the servo on pin 9
  myservo.write(85); // Initial servo position

  time = millis(); // Initialize the time variable
}

void loop() {
  if (millis() > time + period) { // Check if the period has elapsed
    time = millis(); // Update current time
    distance = get_TFminiPlus_distance(); // Read distance from LiDAR
    distance_error = distance_setpoint - distance; // Calculate error

    // Proportional term
    PID_p = kp * distance_error;

    // Derivative term
    float dist_difference = distance_error - distance_previous_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);

    // Integral term
    if (-3 < distance_error && distance_error < 3) {
      PID_i = PID_i + (ki * distance_error);
    } else {
      PID_i = 0;
    }

    // Calculate total PID output
    PID_total = PID_p + PID_i + PID_d;

    // Clamp PID output
    if (PID_total < -30) {
      PID_total = -30;
    }

    // Map PID output to servo range
    Serial.print(PID_total);
    Serial.print("\t");
    PID_total = map(PID_total, -500, 500, 0, 150);

    // Set servo position
    Serial.print(PID_total);
    myservo.write(PID_total);

    // Update previous error
    distance_previous_error = distance_error;

    // Print distance and servo angle for debugging
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Servo Angle: ");
    Serial.println(PID_total + 30);
  }
}
