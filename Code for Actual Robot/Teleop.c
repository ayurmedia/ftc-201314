#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     archemedes,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     flag_raiser,   tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    auto_block,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    blocker,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    bucket,               tServoStandard)

/*
Manipulators inlcuded:
	* Four drivetrain motors wired to two ports
	* One motor for circular torque (flag raiser)
	* One motor for hang (archemedes screw)
	* One servo for the flicker (scoring mechanism for autonomous)
	* One servo for the blocker (defense strategy manipulator)
*/


#include "JoystickDriver.c"


const int DEADZONE = 7;
const float JOY_MAX_VAL = 127.0;
const int NUDGE_POWER = 15;
const int SERVO_MIN = 0, SERVO_MAX = 255, SERVO_DIFF = 20;

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
  //servo[auto_block] = 0;

  p_dtl->id = dt_left;
  p_dtl->MAX_POWER = 80;
  p_dtl->output = 0;

  p_dtr->id = dt_right;
  p_dtr->MAX_POWER = 80;
  p_dtr->output = 0;

  p_fr->id = flag_raiser;
  p_fr->MAX_POWER = 90;
  p_fr->output = 0;

  p_as->id = archemedes;
  p_as->MAX_POWER = 50;
  p_as->output = 0;
/*
  p_ll->id = bucket;
  p_ll->MAX_POWER = 80;
  p_ll->output = 0;
  */

  servo[auto_block] = 0;
  servoChangeRate[auto_block] = 0;

  servo[blocker] = 0;
  servoChangeRate[blocker] = 0;

  servo[bucket] = 0;
  servoChangeRate[bucket] = 0;
}

void hardReset() {
	motor[dt_left] = 0;
	motor[dt_right] = 0;
	motor[archemedes] = 0;
	motor[flag_raiser] = 0;

	servo[auto_block] = 0;
	servo[blocker] = 0;
	servo[bucket] = 0;
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
	if(p_dtr->output == 0) {
		if(joy1Btn(6)) {
			p_dtr->output = NUDGE_POWER;
		} else if(joy1Btn(8)) {
			p_dtr->output = -NUDGE_POWER;
		}
	}

	if(p_dtl->output == 0) {
		if(joy1Btn(5)) {
			p_dtl->output = NUDGE_POWER;
		} else if(joy1Btn(7)) {
			p_dtl->output = -NUDGE_POWER;
		}
	}
}

void buttonMotor(Actuator* p_act, bool p_down, bool p_up) {
	if(p_act->output == 0) {
		if(!p_up && !p_down) {
			p_act->output = 0;
		} else if(p_up) {
			p_act->output = p_act->MAX_POWER;
		} else if(p_down) {
			p_act->output = -p_act->MAX_POWER;
		}
	}
}


void servoOutput(short p_id, bool p_down, bool p_up) {
	int output = ServoValue[p_id];

	if(p_down==true) {
		if(ServoValue[p_id]-SERVO_DIFF < SERVO_MIN) {
			output = SERVO_MIN;
			} else {
			output = ServoValue[p_id] - SERVO_DIFF;
			}
	}else if(p_up==true) {
		if(ServoValue[p_id]+SERVO_DIFF > SERVO_MAX) {
			output = SERVO_MAX;
		} else {
			output = ServoValue[p_id] + SERVO_DIFF;
		}
	} else {
		output = ServoValue[p_id];
	}

	servo[p_id] = output;
}

// Program's entry point; includes main loop
task main() {
	// Drivetrain left, drivetrain right, flag raiser, archemedes screw
	Actuator dtl, dtr, fr, as, ll;

	//initializeRobot(&dtl, &dtr, &fr, &as, &ll);
	initializeRobot(&dtl, &dtr, &fr, &as);
  // waitForStart();

  for(;/*ever*/;) {
  	// Get inputs
	  getJoystickSettings(joystick);

	  // Process inputs
	  scaleJoyValue(&dtl, joystick.joy1_y1);
	  scaleJoyValue(&dtr, joystick.joy1_y2);
	  scaleJoyValue(&as, joystick.joy2_y1);
	  scaleJoyValue(&fr, joystick.joy2_y2);

	  nudgeDrive(&dtl, &dtr);

	  //buttonMotor(&fr, joy1Btn(2), joy1Btn(3));

	  // Output
	  motor[dtl.id] = dtl.output;
	  motor[dtr.id] = dtr.output;
	  motor[fr.id] = fr.output;
	  motor[as.id] = as.output;
	  //motor[ll.id] = ll.output;

	 	servoOutput(auto_block, joy2Btn(6), joy2Btn(8));
	 	servoOutput(blocker, joy2Btn(5), joy2Btn(7));
	 	servoOutput(bucket, joy2Btn(2), joy2Btn(4));

	  if(joy1Btn(9) || joy2Btn(9)) {
	  	hardReset();
		}
  }
}
