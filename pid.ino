double sample_time;
double kp;
double ki;
double kd;
double ref;

double Iterm = 0;
double last_error = 0;
double last_angle = 0;

void PID_initialization(double p, double i, double d, double r, double low, double high)
{
  kp = p;
  ki = i;
  kd = d;
  
  ref = r;

  low = 0;
  high = 135;
}

void set_pid_tuning(double sampling_time)
{
  sample_time = sampling_time;
  
  //if(sample_time > 0 )
  //{
  //  ki *= (sample_time/1000);
  //  kd /=  (sample_time/1000);
  //}
}

// PID controller function
double pid_compute(double input, unsigned long last_time)
{
  double error, diff, output;
  double Pterm, Dterm;
  unsigned long time_change, current_time;
  double ds;

  ds = sample_time/1000;
  
  current_time = millis();
  time_change = current_time - last_time;

  double angle_change = last_angle - input;
//  if(abs(angle_change) > 20)
//  {
//    input = last_angle;
//  }
//
//  if(time_change >= sample_time)
//  {
    
    Iterm += ds * last_error;
    Iterm = ki * Iterm;
  
    // if(Iterm > max_limit) Iterm = max_limit;
    // else if(Iterm < -max_limit) Iterm = -max_limit;
    
    error = ref - input;
    
    // Pterm calculation:
    Pterm = kp*error;
    
    // error difference
    diff = (error - last_error)/ds;

    // Dterm calculation:
    Dterm = kd*diff;
  
    // Compute PID output
    output = Pterm + Iterm + Dterm;
    
    //if(output > max_limit) output = max_limit;
    //else if(output < -max_limit) output = -max_limit;

    //Serial.print("Iterm: ");
    //Serial.println(Iterm);
    //Serial.print("Pterm: ");
    //Serial.println(Pterm);
    //Serial.print("Dterm: ");
    //Serial.println(Dterm);
    
    
    last_error = error;
    time_val = current_time;
    last_angle = input;
    

//    Serial.print("Angle: ");
//    Serial.println(input);
//    Serial.print("PID_output: ");
//    Serial.println(output);
//    
    return output;
//  }

}
