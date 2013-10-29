#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Sensor, S3,     light,          sensorLightActive)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag_raiser,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     archemedes,    tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    auto_block,           tServoStandard)
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
static const int LIGHT_TOLERANCE = 5;


// Positions servo for block-dispensing manipulator
void initializeRobot() {
	servo[auto_block] = 0;

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
    k_followLine,
    k_getOnRamp,
    k_turn90,
    k_finish
  } State;
  State m_state = k_findColoredLine;

  while(m_state != k_finish) {
  	switch(m_state) {
  		case k_findColoredLine:
  			// *** May use encoders
  			// IF: found colored line, m_state++;
  			// ELSE: drive straight
  		break;

  		case k_findIR:
  			// IF: found/centered IR beacon, STOP DRIVETRAIN then m_state++;
  			// ELSE; follow line & drive forward
  		break;

  		case k_dispenseBlock:
  			// IF: servo at certain degree, m_state++;
  			// ELSE: move servo
  		break;

  		case k_followLine:
  			// IF: colored line ends, m_state++;
  			// ELSE: light sensor line-following
  		break;

  		case k_getOnRamp:

  		break;

  		case k_turn90:

  		break;
  	}
	}

	// Speical stupid RobotC thing to keep programming running until end of autonomous period
  for(;/*ever*/;) {}
}
