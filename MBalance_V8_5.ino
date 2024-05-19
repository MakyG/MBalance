// ##################################################
//
// PROJECT:         MBalance
// DESCRIPTION:
//  Program for controlling tow wheeled self balancing robot via PID controller.
//
// AUTHOR:
// Tomas Makyca
// For educational purposes - Tomas Bata University - Zlin
// Tomas Bata University, Zlin - Czech republic. 2023


// \\ ______________________________________________
//                 VERSION BLOG
// V8.4     |to 14. 3. 2024
// RobotSpeed included
// V8.3     | 12. 1. 2024
//          removed unnecesary code + LCD
//          function joystick() enabled + removed previous communication code
//
// V8.2     | 8. 7. 2023 - 5. 9. 2023
// V8.1     | 2. 7. 2023
//
// V8       | 5. 12. 2022
//          switched from Arduino Leonardo (Destroyed) to Arduino Nano (has only 1 port timer so subsequence exists)
//
// V7       | 23. 11. 2022
//          incorrect microstepping, switching Arduino type is required
//          problems with interrupts
//
// V4       | 21. 11. 2022
//          joystick tests
//
// V3       | 21. 11. 2022
// V2       | 21. 11. 2022
// V1       | 12. 11. 2022
//          Arduino Leonardo
//          initial version - inspiration from default project


////________________________________________________\\ 
//              INCLUDED LIBRARIES

#include <TimedAction.h>  // Bluetooth com // V1
#include <Wire.h>         // including PID + Control // V2
#include <I2Cdev.h>       // I2Cdev lib from www.i2cdevlib.com // V7

////_______________________________________________________________________________________________________________________________________

////________________________________________________\\ 
//                     DEFINES

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 20
#define MAX_STEERING 5
#define MAX_TARGET_ANGLE 3

// Default control terms for EVO 2
#define KP 1.2              // 1.2 def
#define KD 0.08             // 0.08 def
#define KP_PI 0.1           // 0.08 slow incorrect finish // 1.2 def            //## FOR RETURNING TO 0 ANGLE
#define KI_PI 0.05          // 0.1 slow reaction // 0.3 def                    //## FOR RETURNING TO 0 ANGLE
#define KP_PI_THROTTLE 0.2  //## FOR CONTROLLING
#define KI_PI_THROTTLE 0.1  //## FOR CONTROLLING
#define KP_POSITION 1.2     // 1.2 def
#define KD_POSITION 0.8     // 0.08 def

#define MAX_CONTROL_OUTPUT 80  // WAS 800
#define ITERM_MAX_ERROR 30     // Iterm windup constants for PI control
#define ITERM_MAX 10000
#define ANGLE_OFFSET 0.82  // Offset angle for balance (to compensate robot own weight distribution) // V8.1

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE 0
#define BATT_VOLT_FACTOR 8  // V3
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 65535
#define MAX_ACCEL 20      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)
#define MICROSTEPPING 16  // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)
#define DEBUG 0           // 0 = No debug info (default) DEBUG 1 for console output

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define BATT_VOLT_FACTOR 8

int16_t motor[2];
int16_t actualMotorSpeed[2];  // actual speed of motors
uint8_t actualMotorDir[2];    // actual direction of steppers motors


uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;         // To generate a medium loop 40Hz
uint8_t slow_loop_counter;    // slow loop 2Hz
uint8_t sendBattery_counter;  // To send battery status
int16_t BatteryValue;

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_PI = KP_PI;
float Ki_PI = KI_PI;
float Kp_PI_thr = KP_PI_THROTTLE;
float Ki_PI_thr = KI_PI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

bool positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;  // Actual speed of motors
int8_t dir_M1, dir_M2;       // Actual direction of steppers motors
int16_t actual_robot_speed;  // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;  // Estimated robot speed

// V6 remove in V7
uint8_t OSCpush[4];
uint8_t OSCmove_mode;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;

int val, cnt = 0, v[4];
int pos1 = 0, pos2 = 0;
int val1, val2, val3, val4;
float steerValue = 0;
float throttleValue = 0;
int timer1;
int Ok1 = 1;
int delayx1;
// V3
int battery;
int maxControlOutput = MAX_CONTROL_OUTPUT;
bool PCM = false;
int counterTest = 0;


////________________________________________________\\ 
//                  MULTITHREADING
void program() {
  timer1++;
}
TimedAction readbtThread = TimedAction(10, readbt);
TimedAction timerThread = TimedAction(1, program);

////________________________________________________\\ 
//                  PROGRAM SETUP

void setup() {
  Serial.begin(9600);
  //_______________________________________SETTING PORTS MODE
  pinMode(3, OUTPUT);  // ENABLE MOTORS 1
  pinMode(4, OUTPUT);  // ENABLE MOTORS 2
  pinMode(5, OUTPUT);  // STEP MOTOR 1   PORTD5
  pinMode(6, OUTPUT);  // STEP MOTOR 2    PORTD6 / problem
  pinMode(7, OUTPUT);  // DIR MOTOR 1    PORTD7
  pinMode(8, OUTPUT);  // DIR MOTOR 2     PORTB0        

  ////_________________________________________________________________
  /********  Specific Timers & Registers for the ATmega328P   ********/
  
  digitalWrite(3, HIGH);  // Disable motor 1
  digitalWrite(4, HIGH);  // Disable motor 2
  digitalWrite(5, LOW);   // Disable motors
  digitalWrite(6, LOW);   // Disable motors

  // Overwrite the Timer1 to use the stepper motors

  //_______________________________________STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);
  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 80;  // 25Khz
  TCNT1 = 0;

  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 interrupt


  //________________________________________________
  //                  STARTUP
  Serial.begin(115200);  // Serial output to console

  //_______________________________________I2C bus INIT
  Wire.begin();
  delay(500);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU
  delay(200);
  MPU6050_calibrate();  // V8.1 // ________CALLIBRATION

  //_______________________________________STEPPER MOTORS INITIALIZATION
  Serial.println("Stepers init");
  delay(200);
  digitalWrite(3, LOW);  // Enable stepper driver 1
  digitalWrite(4, LOW);  // Enable stepper driver 2

  // Little motor vibration to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++) {
    setMotorSpeed(0, 5);
    setMotorSpeed(1, 5);
    delay(100);

    setMotorSpeed(0, -5);
    setMotorSpeed(1, -5);
    delay(100);
  }

  Serial.println("Start...");
  timer_old = micros();
  Serial.println("OK");

  // set default settings
  steps1 = 0;
  target_steps1 = 0;
}

// V1
void joystick1() {

  // CODING
  // 1 = CONTROL VALUES
  // 2 = OSC SWITCH
  if (Serial.available()) {
    while (Serial.available() == 0)
      ;
    readbtThread.check();
    timerThread.check();
    if (v[1] == 1) {  // position joystick change
      // IS CURRENTLY STEERING/THROTTLING
      // V6

      val2 = v[3];  // THROTTLE
      throttleValue = ((float)val2 - 100) / 100;

      val1 = v[2];  // STEER
      steerValue = ((float)val1 - 100) / 10;
    }

    if (v[1] == 2) {  // switch PCM

      if (v[2] == 1) {
        Serial.println("PCM ON");
        PCM = true;
      }
      if (v[2] == 0) {
        Serial.println("PCM OFF");
        PCM = false;
      }
    }

    if (v[1] == 3) {  // switch motorsEnabled

      if (v[2] == 1) {
        Serial.println("Motors enabled");
        maxControlOutput = MAX_CONTROL_OUTPUT;
      }
      if (v[2] == 0) {
        Serial.println("Motors disabled");
        maxControlOutput = 0;
      }
    }
  }
}

////________________________________________________\\ 
//                  COMMUNICATION

void readbt() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (v[1] == 1 && cnt == 3) {
    cnt = 0;
  }
  if (v[1] != 1 && cnt == 2) {
    cnt = 0;
  }
}

//// ______________________________________________________________________________________ START OF MAIN CODE
////________________________________________________\\ 
//                  PROGRAM LOOP
void loop() {

  //________________________________________________
  //      HANDLING INPUT FROM MOBILE DEVICE
  joystick1();

  if (steering > 0)
    steering = steerValue / 10;
  else {
    //steering = 0;
  }

  timer_value = micros();

  //________________________________________________New IMU data REACTION
  if (MPU6050_newData()) {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001;  // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;

    // Get new orientation angle from IMU (MPU6050):
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;

    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
      angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

    // Calculate the estimated robot speed:
    // FIXED IN V8.4
    actual_robot_speed = (actualMotorSpeed[0] + (-actualMotorSpeed[1])) / 2;  // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0;  // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = -(estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1);  // low pass filter on estimated speed


    // ###
    // ### _______________P I D regulator___________________ ###
  
    // ##### IF HOMING #####

    if (!PCM) {
      
      target_steps1 = steps1;// + OSCmove_steps1;
      //target_steps2 = steps2 + OSCmove_steps2;

      // ## _______ Position Control: PD controller
      //    input: Target steps for each motor, output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position*1.6, Kd_position*1.6, actualMotorSpeed[0]);
      //motor2_control = positionPDControl(steps2, target_steps2, Kp_position*1, Kd_position*2, actualMotorSpeed[1]);
      
      // Convert from motor position control to throttle / steering commands
      throttle = motor1_control / 2;
      throttle = constrain(throttle, -50, 50);
      //steering = motor2_control - motor1_control;
      //steering = constrain(steering, -50, 50);

    } else{
      //steps1 = 0;
      //target_steps1 = 0;
      motor1_control = 0;
      throttle = throttleValue * 30; // REMOTE CONTROL
    }

    // ## _______  Speed Control: PI controller
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, -estimated_speed_filtered, throttle, Kp_PI*0.8, Ki_PI*1.2);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle);  // limited output

    // ## _______  Stability control: PD controller
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    integration of the output (sumatory) = output is the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp*0.4, Kd);
    control_output = constrain(control_output, -maxControlOutput, maxControlOutput);  // Limit max output from control

    // -> Apply control_output to the motors
    if (!PCM){
      motor1 = control_output;
      motor2 = -(control_output);
    } else{
      motor1 = control_output - steerValue;
      motor2 = -(control_output)-steerValue;
    }

    // MAX SPEED LIMIT
    motor1 = constrain(motor1, -maxControlOutput, maxControlOutput);
    motor2 = constrain(motor2, -maxControlOutput, maxControlOutput);

    int angle_ready;
    angle_ready = 15;  // Default angle
    
    // ROBOT READY STATUS
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready))  
    {
      // NORMAL MODE
      digitalWrite(3, LOW);     // Motor 1 enable
      digitalWrite(4, LOW);     // Motor 2 enable
      setMotorSpeed(0, motor1);
      setMotorSpeed(1, motor2);
    } else{
      // Robot not ready (flat), angle > angle_ready => ROBOT OFF
      digitalWrite(3, HIGH);    // Motor 1 disable
      digitalWrite(4, HIGH);    // Motor 2 disable
      setMotorSpeed(0, 0);
      setMotorSpeed(1, 0);

      PID_errorSum = 0;  // Reset PID I term
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      OSCmove_mode = false;
      throttle = 0;
      //steering = 0;
    }
  }  // End of new IMU data

  // Medium loop - | 10 Hz |
  if (loop_counter >= 10) {
    loop_counter = 0;
    Serial.print(timer_value * 0.000001);
    Serial.print(";");

    Serial.print(target_angle);
    Serial.print(";");
    Serial.print(control_output);
    Serial.print(";");
    Serial.print(angle_adjusted);
    Serial.print(";");
    Serial.print(throttle);

    Serial.print(";");
    Serial.print(steerValue);
    
    Serial.print(";");
    Serial.print(estimated_speed_filtered);
    Serial.print(";");
    Serial.print(steps1);
    Serial.print(";");
    /*
    Serial.print(motor1_control);
    Serial.print(";");
    */

    Serial.println("");

    if (counterTest > 160){
      counterTest = 0;
    } 
    else if (counterTest > 80) {
      counterTest++;
    }
    else{
      counterTest++;
    }

    }                   
  // Slow loop - | 1 Hz |                
  else if (slow_loop_counter >= 100)
  {
    throttle = 5;
    
    slow_loop_counter = 0;
  }
}


// ##
// PD controller definition (Proportional, derivative). DT in seconds (V2)
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd) {
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8);  // limit the input part
  
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;

  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;

  return (output);
}


// ##
// PI controller definition (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki) {
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  output = Kp * error + Ki * PID_errorSum * DT;  // DT is in miliseconds

  return (output);
}


// ##
// PD controller definition (Proportional, derivative). DT in seconds
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM) {
  float error;
  float output;
  float P;

  error = float(setPointPos - actualPos);
  P = constrain(Kpp * error, -115, 115);  // Limit command

  output = P + Kdp * float(speedM);

  return (output);
}

//// ______________________________________________________________________________________ END OF MAIN CODE



/******************#   Calculate and write the motors values   #******************/

uint16_t periodsCounter[2];  // counters for periods
uint16_t subPeriod[2][8];    // eight subperiodPaddings
uint8_t subPeriodIndex[2];   // index for subperiodPaddings

#define ZERO_SPEED 65535
int16_t speed;


// Divided into 8 sub-periods to increase the resolution at high speeds (short periods)
// subperiodPadding = ((1000 % vel)*8)/vel;
void calculateSubperiods(uint8_t motor) {

  uint8_t subperiodPadding;
  uint16_t absSpeed;
  uint8_t i;

  if (actualMotorSpeed[motor] == 0) {
    for (i = 0; i < 8; i++) {
      subPeriod[motor][i] = ZERO_SPEED;
    }
    return;
  }


#ifdef REVERSE_MOTORS_DIRECTION
  actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 1 : 0;
#else
  actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 0 : 1;
#endif

#if MICROSTEPPING == 16
  speed = actualMotorSpeed[motor] * 6;  // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = actualMotorSpeed[motor] * 3;  // 1/8 Microstepping
#endif

  absSpeed = abs(speed);

  subPeriod[motor][0] = 1000 / absSpeed;
  for (i = 1; i < 8; i++) {
    subPeriod[motor][i] = subPeriod[motor][0];
  }
  // Calculate the sub-period padding.
  subperiodPadding = ((1000 % absSpeed) * 8) / absSpeed;
  if (subperiodPadding > 0) {
    subPeriod[motor][1]++;
  }
  if (subperiodPadding > 1) {
    subPeriod[motor][5]++;
  }
  if (subperiodPadding > 2) {
    subPeriod[motor][3]++;
  }
  if (subperiodPadding > 3) {
    subPeriod[motor][7]++;
  }
  if (subperiodPadding > 4) {
    subPeriod[motor][0]++;
  }
  if (subperiodPadding > 5) {
    subPeriod[motor][4]++;
  }
  if (subperiodPadding > 6) {
    subPeriod[motor][2]++;
  }
}

void setMotorSpeed(uint8_t motorNum, int16_t tspeed) {

  // LIMIT MAX ACCELERATION of the motors
  if ((actualMotorSpeed[motorNum] - tspeed) > MAX_ACCEL)
    actualMotorSpeed[motorNum] -= MAX_ACCEL;
  else if ((actualMotorSpeed[motorNum] - tspeed) < -MAX_ACCEL)
    actualMotorSpeed[motorNum] += MAX_ACCEL;
  else
    actualMotorSpeed[motorNum] = tspeed;

  calculateSubperiods(motorNum);  // We use four subperiodPaddings to increase resolution

  // Energy saving
  if ((actualMotorSpeed[0] == 0) && (actualMotorSpeed[1] == 0)) {
    //digitalWrite(4,HIGH);   // Disable motor 1
    //digitalWrite(3,HIGH);   // Disable motor 2
  } else {
    digitalWrite(3, LOW);  // Enable motor 1
    digitalWrite(4, LOW);  // Enable motor 2
  }
}



/****   ISR   ****/
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))

ISR(TIMER1_COMPA_vect)  // DRIVER_PIN[5] = {5,6,7,8,4};   //STEP1 (PORTD 5), STEP2 (PORTD 6), DIR1 (PORTD 7), DIR2 (PORTB 0), ENABLE
{
  periodsCounter[0]++;
  periodsCounter[1]++;

  //  ***
  //  DIR1 / STEP1 _ MOTOR1
  if (periodsCounter[0] >= subPeriod[0][subPeriodIndex[0]]) {
    periodsCounter[0] = 0;

    if (subPeriod[0][0] != ZERO_SPEED) {
      if (actualMotorDir[0]) {
        SET(PORTD, 7);  // DIR Motor 1 / pin 7
        steps1--;
      } else {
        CLR(PORTD, 7);
        steps1++;
      }
      // We need to wait at lest 200ns to generate the Step pulse...
      subPeriodIndex[0] = (subPeriodIndex[0] + 1) & 0x07;  // subPeriodIndex from 0 to 7

      SET(PORTD, 5);  // STEP Motor 1 / pin 5
      delayMicroseconds(1);
      CLR(PORTD, 5);
    }
  }

  //  ***
  //  DIR2 / STEP2 _ MOTOR2
  if (periodsCounter[1] >= subPeriod[1][subPeriodIndex[1]]) {
    periodsCounter[1] = 0;

    if (subPeriod[1][0] != ZERO_SPEED) {

      if (actualMotorDir[1]) {
        SET(PORTB, 0);  // DIR Motor 2 / pin 8
      } else {
        CLR(PORTB, 0);
      }
      subPeriodIndex[1] = (subPeriodIndex[1] + 1) & 0x07;

      SET(PORTD, 6);  // STEP Motor 2 / pin 6
      delayMicroseconds(1);
      CLR(PORTD, 6);
    }
  }
}
