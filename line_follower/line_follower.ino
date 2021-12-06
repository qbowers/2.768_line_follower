const int PWMA=11,
          AIN2=10,
          AIN1=9,
          STDBY=8,
          BIN1=7,
          BIN2=6,
          PWMB=5,

          PID_LED=4,

          SL=A0,
          SM=A1,
          SR=A2;



void setup() {
  //Motor Controller Setup
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(STDBY,OUTPUT);
  digitalWrite(STDBY, HIGH);

  pinMode(PID_LED, OUTPUT);
  digitalWrite(PID_LED, LOW);

  //IR Sensor Setup
  pinMode(SL, INPUT);
  pinMode(SM, INPUT);
  pinMode(SR, INPUT);

  //Serial Setup
  Serial.begin(9600);

  set_initial_weights();
  // set_rough_weights();
}

void loop() {
  static float loop_time = 0.1;
  static int start_time_micros = 0;
  
  //Read and normalize sensor values
  //High value -> darker line under sensor
  float NSL = map(analogRead(SL),200,400,0.0,100.0);
  float NSM = map(analogRead(SM),150,400,0.0,100.0);
  float NSR = map(analogRead(SR),230,400,0.0,100.0);
  
  
  loop_time = (start_time_micros-micros()) / 1000000.0;
  start_time_micros = micros();

  // sensorPrint(NSL, NSM, NSR);
  // drive_straight();
  PID_center_control(NSL, NSM, NSR, loop_time);
  // bang_control(NSL, NSM, NSR, loop_time);

  delay(10);
}


void PID_control(float NSL, float NSM, float NSR, int loop_time) {
  
  static float spd = 90,    //default motor speed
             steer = 0,     //The control action. This is what the controller changes
             e = 0,         //This is how wrong we are
             e_prev = e,    //store the error from the last cycle, for derivative calculation
             e_int = 0,     //integral of the error
             e_deriv = 0,   //derivative of the error
             alpha = 0.25;  //filter coefficient (don't worry about it)
  
  //compute error
  //positive error means turn left
  //(error correlated to x-position)
  /*
     o|o o    |   < negative error
      | o o o |   < no error
      |    o o|o  < positive error
  */
  e = NSL - NSR;

  //Take the derivative of error, then put it into a 1st order filter
  e_deriv = alpha*(e - e_prev)/loop_time  + (1-alpha)*e_deriv;
  //integrate the error over time. constrain it to avoid windup
  e_int += constrain(e * loop_time, -10000, 10000); 

  //store error for next time
  e_prev = e;


  //controller coefficients
  static int  Kp = 10,
              Kd = 5,
              Ki = 10;

  //output. constrain to set a max turn radius
  steer = constrain( Kp*e + Kd*e_deriv + Ki*e_int , -75, 75);
  
  drive(spd - steer, spd + steer);
}

//controller coefficients
float Kp, //0.000037,
      Kd,//1.2,
      Ki;
void PID_center_control(float NSL, float NSM, float NSR, int loop_time) {
  static float  spd = 255,    //default motor speed ---------- max is 255
                steer = 0,     //The control action. This is what the controller changes
                e = 0,         //This is how wrong we are
                e_prev = e,    //store the error from the last cycle, for derivative calculation
                e_int = 0,     //integral of the error
                e_deriv = 0,   //derivative of the error

                alpha = 0.25,  //filter coefficient (don't worry about it)

                threshold = 7,
                steer_deadzone = 2,
                e_int_max = 100000,

                
  
                mid_range_scale = 5.0;
  static bool right = false;


  int LR_diff = NSL - NSR; // high -> Left darker than Right -> too far right -> positive error
  int LM_diff = NSL - NSM; // high -> Left darker than Middle -> too far right -> large positive error
  int MR_diff = NSM - NSR; // low -> Right darker than Middle -> too far left -> large negative error

  //compute error  (error correlated to x-position) (positive error means turn left)
  /*
   o o|o      |    <- large negative error
     o|o o    |    <- negative error
      | o o o |    <- no error
      |    o o|o   <- positive error
      |      o|o o <- large positive error
          .
          .
          .
         | |
       o o o        <- small negative error
        o|o|o       <- no error
         o o o      <- small positive error
         | |
  */

  if (LR_diff > threshold) {
    //too far right -> positive error
    right = true;
    // e = LR_diff + mid_range_scale*LM_diff;
    e = LR_diff + LM_diff*abs(LM_diff)/5.0;
  } else if (LR_diff < -threshold) {
    //too far left -> negative error
    right = false;
    // e = LR_diff + mid_range_scale*MR_diff;
    e = LR_diff + MR_diff*abs(MR_diff)/5.0;
  } else {
    //deadzone
    e = 0;
  }

  // Do Special Case if all sensors are white
  static int white_space_number = 0,
             white_threshold = 80;
  static float white_time = 0,
               black_time = 0;
  static float white_time_threshold = 0.6 * 10000000.0/spd,//150.0/spd, //should be lower if we go fast
               black_time_threshold = 0.6 * 10000000.0/spd;//200.0/spd;
  static bool is_white = false;

  sensorPrint(white_space_number,0,0);
  if (NSL < white_threshold && NSM < white_threshold-20 && NSR < white_threshold) {
    // Read all white
    white_time += loop_time;
    black_time = 0;
    if (white_time >= white_time_threshold ) {
      if (!is_white) {
        // black_time = 0;
        is_white = true;
        white_space_number += 1;
      }
    }
  } else {
    // Did not read all white
    black_time += loop_time;
    white_time = 0;
    if (black_time >= black_time_threshold) {
      is_white = false;
      // white_time = 0;
    }
  }

  if (is_white) {
    digitalWrite(PID_LED, HIGH);
    e_int = 0;
    if (drive_white(white_space_number)) return;
  } else {
    digitalWrite(PID_LED, LOW);
  }



  //Take the derivative of error, then put it into a 1st order filter
  e_deriv = alpha*(e - e_prev)/loop_time  + (1-alpha)*e_deriv;
  //integrate the error over time. constrain it to avoid windup
  e_int = constrain(e_int + (e * loop_time), -e_int_max, e_int_max); 

  //store error for next time
  e_prev = e;

  //output
  // steer = Kp*e*abs(e)*abs(e) + Kd*e_deriv + Ki*e_int;
  steer = Kp*e + Kd*e_deriv + Ki*e_int; //in the range -500, 500

  // sensorPrint(e, steer, 0);
  static int spd_2 = spd;
  if (abs(Ki*e_int) > 200) {
    spd_2 = spd - (steer) / 4.0;
  } else {
    spd_2 = spd;
  }


  if (steer > 0) {
    drive(spd_2 - steer, spd_2);
  } else {
    drive(spd_2, spd_2 + steer);
  }
  //dead zone
  if (abs(steer) < steer_deadzone) {
    drive(spd, spd);
  }
}
void set_initial_weights() {
  Kp = 0.8, //0.000037,
  Kd = 1.0,//1.2,
  Ki = 0.0003;
}
void set_smooth_weights() {
  Kp = 0.4;
  Kd = 0.4;
  Ki = 0;
}
void set_rough_weights() {
  Kp = 0.25;
  Kd = 0;
  Ki = 0.0000001;
}
bool drive_white(int white_space_number) {
  switch(white_space_number+1) {
    case 2: {
      drive_straight();
      set_smooth_weights();
      break;
    }
    case 3: {
      drive_straight();
      set_initial_weights();
      break;
    }
    case 4: {
      turn_left();
      break;
    }
    case 5: {
      turn_right();
      break;
    }
    case 6: {
      turn_left();
      set_rough_weights();
      break;
    }
    case 7: {
      turn_left();
      set_rough_weights();
      break;
    }
    default: {
      turn_left(); //do pid instead
      break;
    }

  }
  return true;
}
void turn_right() {
  drive(255,-100);
}
void turn_left() {
  drive(-100,255);
}
void drive_straight() {
  drive(255,255);
}

void bang_control(float NSL, float NSM, float NSR, int loop_time) {
  if (NSR-NSL > 20) {
    drive(100,25);
  } else if (NSL-NSR>20) {
    drive(25,100);
  } else {
    drive(100,100);
  }
}

void sensorPrint(float L, float M, float R) {
  Serial.print(L);
  Serial.print(" ");
  Serial.print(M);
  Serial.print(" ");
  Serial.println(R);
}

void motorWrite(int spd, int pin_IN1, int pin_IN2, int pin_PWM) {
  if(spd<0)
  {
    digitalWrite(pin_IN1,HIGH);
    digitalWrite(pin_IN2,LOW);
  }
  else
  {
    digitalWrite(pin_IN1,LOW);
    digitalWrite(pin_IN2,HIGH);
  }
  analogWrite(pin_PWM, abs(spd));
}

void drive(int speedL, int speedR) {
  motorWrite(speedR, AIN1, AIN2, PWMA);
  motorWrite(speedL,BIN1, BIN2, PWMB);
}
