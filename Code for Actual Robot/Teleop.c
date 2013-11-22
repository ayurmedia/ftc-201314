#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag_raiser,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     archemedes,    tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    auto_block,           tServoStandard)

#include "JoystickDriver.c"


const int DEADZONE = 7;
const float JOY_MAX_VAL = 127.0;
const int NUDGE_POWER = 25;

// Data structure to try to keep things object-oriented and modular
// Maps every motor to its #pragma, its max power, and its output during every loop
// Eliminates the need to keep multiple variables for each motor
typedef struct {
	short id;
	int MAX_POWER;
	int output;
} Actuator;


// Sets servo's starting position
// Initializes all fields for "Actuators"
void initializeRobot(Actuator* p_dtl, Actuator* p_dtr, Actuator* p_fr, Actuator* p_as) {
  servo[auto_block] = 0;

  p_dtl->id = dt_left;
  p_dtl->MAX_POWER = 80;
  p_dtl->output = 0;

  p_dtr->id = dt_right;
  p_dtr->MAX_POWER = 80;
  p_dtr->output = 0;

  p_fr->id = flag_raiser;
  p_fr->MAX_POWER = 80;
  p_fr->output = 0;

  p_as->id = archemedes;
  p_as->MAX_POWER = 50;
  p_as->output = 0;
}

// Converts raw joystick values, which range from -128 to 127, into motor powers
int scaleJoyValue(Actuator* p_act, float p_joy_val) {
	// If the joystick value is the within the deadzone, do not move motors
	if(abs(p_joy_val) < DEADZONE) {
		p_act->output = 0;
	}
	// In all other cases, to scale the joystick value into motor value,
		// use an algorithm of function m(j) = (j^2)/(JM^2) * MM,
		// where m = motor output, j = raw joystick value, JM = JOY_MAX_VAL, and MM = specific motor's MAX_POWER.
	// HOWEVER, squaring the raw joystick value would eliminate the sign (+/-),
		// so extra precaution need be taken to preserve the direction.
	else {
		int direction = p_joy_val / abs(p_joy_val);
		p_act->output =  direction * (p_joy_val*p_joy_val) / (JOY_MAX_VAL*JOY_MAX_VAL) * p_act->MAX_POWER;
	}

	return p_act->output;
}

void nudgeDrive(Actuator* p_dtl, Actuator* p_dtr) {
	if(p_dtl->output == 0) {
		if(joy1Btn(7)) {
			p_dtl->output = NUDGE_POWER;
		} else if(joy1Btn(5)) {
			p_dtl->output = -NUDGE_POWER;
		}
	}

	if(p_dtr->output == 0) {
		if(joy1Btn(8)) {
			p_dtr->output = NUDGE_POWER;
		} else if(joy1Btn(6)) {
			p_dtr->output = -NUDGE_POWER;
		}
	}
}

// Program's entry point; includes main loop
task main() {
	// Drivetrain left, drivetrain right, flag raiser, archemedes screw
	Actuator dtl, dtr, fr, as;

	initializeRobot(&dtl, &dtr, &fr, &as);
  // waitForStart();

  for(;/*ever*/;) {
  	// Get inputs
	  getJoystickSettings(joystick);

	  // Process inputs
	  scaleJoyValue(&dtl, joystick.joy1_y1);
	  scaleJoyValue(&dtr, joystick.joy1_y2);
	  scaleJoyValue(&fr, joystick.joy2_y1);
	  scaleJoyValue(&as, joystick.joy2_y2);

	  nudgeDrive(&dtl, &dtr);

	  // Output
	  motor[dtl.id] = dtl.output;
	  motor[dtr.id] = dtr.output;
	  motor[fr.id] = fr.output;
	  motor[as.id] = as.output;
  }
}
