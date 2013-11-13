#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Sensor, S3,     light,          sensorLightActive)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag_raiser,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     archemedes,    tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_2,    UNUSED1,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    UNUSED2,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    UNUSED3,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    UNUSED4,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    UNUSED5,              tServoStandard)

#include "JoystickDriver.c"
// Constants for light sensor
static const int GRAY_LIGHT_VAL = 0;
static const int BLUE_LIGHT_VAL = 0;
static const int RED_LIGHT_VAL = 0;
static const int WHITE_LIGHT_VAL = 0;
static const int LIGHT_TOLERANCE = 5;
static const int THRESHOLD = 5;
static const int IR_LOCATION = 5;
static const int MAX_POWER = 80;
static const int MAX_SERVO_ANGLE = 120;
static const int TIME_ON_RAMP = 5;
int timer = 0;

void moveForward()
{
  motor[dt_right] = MAX_POWER;
  motor[dt_left] = MAX_POWER;
}

void moveRight()
{
  motor[dt_right] = -MAX_POWER;
  motor[dt_left] = MAX_POWER;
  wait1MSec(1000);
}

void promptForInputs() {


}


// Positions servo for block-dispensing manipulator
void initializeRobot() {
	servo[auto_block] = 0;
  promptForInputs();


  return;
}

task main() {
  initializeRobot();
  // waitForStart();

  // Lists all phases/states of the autonomous period
  typedef enum {
    k_findColoredLine = 0,
    k_findIR,
    k_dispenseBlock,
    k_followColoredLine,
    k_findWhiteLine,
    k_getOnRamp,
    k_finish
  } State;
  State m_state = k_findColoredLine;

  while(m_state != k_finish) {
  	switch(m_state) {
  		case k_findColoredLine:
  			// IF: found colored line, m_state++;
  			// ELSE: drive straight
        while(SensorValue[light] - GRAY_LIGHT_VAL <= THRESHOLD)
        {
          moveForward();
        }
  		break;

  		case k_findIR:
  			// IF: found/centered IR beacon, STOP DRIVETRAIN then m_state++;
  			// ELSE; follow line & drive forward
      while((SensorValue[light] - BLUE_LIGHT_VAL <= THRESHOLD || SensorValue[light] - RED_LIGHT_VAL <= THRESHOLD) && SensorValue[ir_seeker] != IR_LOCATION)
      {
          moveForward();
      }
      servo[auto_block] = MAX_SERVO_ANGLE;
  		break;

  		case k_dispenseBlock:
  			// IF: servo at certain degree, m_state++;
  			// ELSE: move servo
  		break;

  		case k_followColoredLine:
  			// IF: colored line ends, m_state++;
  			// ELSE: light sensor line-following
      while(SensorValue[light] - BLUE_LIGHT_VAL <= THRESHOLD || SensorValue[light] - RED_LIGHT_VAL <= THRESHOLD)
      {
        moveForward();
      }
      while
  		break;

  		case k_findWhiteLine:
      turnRight();
      while(SensorValue[light] - GRAY_LIGHT_VAL <= THRESHOLD)
      {
        moveForward();
      }
      turnRight();
      while(SensorValue[light] - WHITE_LIGHT_VAL <= THRESHOLD && timer <= TIME_ON_RAMP)
      {
        moveForward();
        timer++;
      }
  		break;

  		case k_getOnRamp:

  		break;
  	}
	}

	// Speical stupid RobotC thing to keep programming running until end of autonomous period
  for(;/*ever*/;) {}
}
