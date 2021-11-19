const int PWMA=11,
          AIN2=10,
          AIN1=9,
          STDBY=8,
          BIN1=7,
          BIN2=6,
          PWMB=5,

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

  //IR Sensor Setup
  pinMode(SL, INPUT);
  pinMode(SM, INPUT);
  pinMode(SR, INPUT);

  //Serial Setup
  Serial.begin(9600);
}

void loop() {
  static float loop_time = 0.1;
  static int start_time = 0;
  
  //Read and normalize sensor values
  //High value -> darker line under sensor
  float NSL = map(analogRead(SL),200,400,0.0,100);
  float NSM = map(analogRead(SM),150,400,0.0,100);
  float NSR = map(analogRead(SR),230,400,0.0,100);
  
  
  loop_time = (start_time-micros()) / 1000000.0;
  start_time = micros();

  sensorPrint(NSL, NSM, NSR);

  PID_control(NSL, NSM, NSR, loop_time);
  // bang_control(NSL, NSM, NSR, loop_time);

  delay(100);
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


void PID_center_control(float NSL, float NSM, float NSR, int loop_time) {
  
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
   o o|o      |    < large negative error
     o|o o    |    < negative error
      | o o o |    < no error
      |    o o|o   < positive error
      |      o|o o < large positive error
  */
  int LR_diff = NSL - NSR;
  int LM_diff = NSL - NSM;  
  int MR_diff = NSM - NSR;

  e = LR_diff + 0;
  if (LR_diff < 0) {
    e += 
  }


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
