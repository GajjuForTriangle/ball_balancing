#include<Servo.h>
Servo servo1;
Servo servo2;
//long duration;
//int distance;
#define trig 2
#define echo 3
#define k_p 0.5
#define k_i 0.2
#define k_d 0
#define ref_val 6.5
#define min_lim 0
#define max_lim 180
double sampling_time=20;
unsigned long time_val;
double input_val;


void PID_initialization(double , double , double, double , double , double );
void set_pid_tuning(double );
double pid_compute(double , unsigned long );
long distance(){
  digitalWrite(trig,LOW);
  delayMicroseconds(4);
  digitalWrite(trig,HIGH);
  delayMicroseconds(4);
  digitalWrite(trig,LOW);
  long t = pulseIn(echo,HIGH);
  double cm=t/(290/2);
  Serial.print("\tcm :");
  Serial.println(cm);
  return cm;
}
//void PID(){
//  int dis=distance();
//  int setP=15;
//  double error=setP-dis;
//
//
//  double Pvalue=error*kp;
//  double Ivalue=toError*ki;
//  double Dvalue=(error-priError)*kd;
//
//  double PIDvalue=Pvalue+Ivalue+Dvalue;
//  priError=error;
//  toError+=error;
////  Serial.println(PIDvalue);
//  int Fvalue=abs(PIDvalue);
//  Serial.print("PID:");
////  Serial.println(Fvalue);
////
//  Fvalue=map(Fvalue,0,135,0,90);
//  Serial.println(Fvalue); 
//  if(PIDvalue<0){
//   servo1.write(Fvalue) ;
//  }
//  if(PIDvalue>135){
//    servo2.write(Fvalue);
//  }
//}
void setup() {
  // put your setup code here, to run once:
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  servo1.attach(9);
  servo2.attach(10);
  Serial.begin(9600);
  servo1.write(0);
  servo2.write(180); 
  delay(500);
  PID_initialization(k_p, k_i, k_d, ref_val, min_lim, max_lim);
  set_pid_tuning(sampling_time);
  
  time_val = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
//  PID();
input_val=distance();
double pid_output = pid_compute(input_val, time_val);
double servo_angle= (pid_output);
Serial.print("\tservo 1\t");
Serial.print(servo_angle);
Serial.print("\tservo 2\t");
Serial.println(180-servo_angle);
if (pid_output<0){
  servo1.write(servo_angle);
  servo2.write(180-servo_angle);

}
else
{
   servo2.write(servo_angle);
  servo1.write(180-servo_angle);
}
}
