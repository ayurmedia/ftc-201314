#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     dt_right,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     dt_left,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     hang,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     flag,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    auto_block,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    bucket,               tServoStandard)

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.



const int DEADZONE = 7;
const float JOY_MAX_VAL = 127.0;
const int NUDGE_POWER = 15;

typedef struct {
	short id;
	int MAX_POWER;
	int output;
} Actuator;


// Sets servo's starting position
// Initializes all fields for "Actuators"
void initializeRobot(Actuator* p_dtl, Actuator* p_dtr, Actuator* p_fr, Actuator* p_as, Actuator* p_ls) {

  p_dtl->id = dt_left;
  p_dtl->MAX_POWER = 80;
  p_dtl->output = 0;

  p_dtr->id = dt_right;
  p_dtr->MAX_POWER = 80;
  p_dtr->output = 0;

  p_fr->id = flag;
  p_fr->MAX_POWER = 90;
  p_fr->output = 0;

  p_as->id = hang;
  p_as->MAX_POWER = 80;
  p_as->output = 0;

  servo[auto_block] = 255;
  servoChangeRate[auto_block] = 0;

  servo[bucket] = 255;
  servoChangeRate[bucket] = 0;
}



void hardReset() {
	motor[dt_left] = 0;
	motor[dt_right] = 0;
	motor[hang] = 0;
	motor[flag] = 0;

	servo[auto_block] = 0;
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
//"Nudges" a drive train motor forward depending on the button pressed
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


//with tims changes
void newSimpleServo() {
	if(joy2Btn(8)) {
		servo[bucket] = 75;
	  //servo[bucket2] = (127-127)+ERROR_VAL;
	}
	else if(joy2Btn(6)) {
		servo[bucket] = 180;
		//servo[bucket2] = (127-0)+ERROR_VAL;
	}
	if(joy2Btn(5)) {
		servo[bucket] = 0;
	}
	if(joy2Btn(7)) {
		servo[bucket] = 40;
	}
	if(joy2Btn(1)) {
		servo[auto_block] = 0;
	}
	if(joy2Btn(4)) {
		servo[auto_block] = 255;
	}
}

//Uses buttons rather than joysticks to move drive train
void buttonMotors(Actuator* p_act, bool p_down, bool p_up) {
		if(p_down) {
			p_act->output = -p_act->MAX_POWER;
		} else if(p_up) {
			p_act->output = p_act->MAX_POWER;
		} else {
			p_act->output = 0;
		}
}

// Program's entry point; includes main loop
task main() {
	// Drivetrain left, drivetrain right, flag raiser, archemedes screw
	Actuator dtl, dtr, fr, as, ls;

	initializeRobot(&dtl, &dtr, &fr, &as, &ls);
  waitForStart();

  for(;/*ever*/;) {
  	// Get inputs
	  getJoystickSettings(joystick);

	  // Process inputs
	  scaleJoyValue(&dtl, joystick.joy1_y1);
	  scaleJoyValue(&dtr, joystick.joy1_y2);
	  scaleJoyValue(&ls, joystick.joy2_y1);
	  scaleJoyValue(&as, joystick.joy2_y2);

	  nudgeDrive(&dtl, &dtr);

	  buttonMotors(&fr, joy1Btn(2) || joy2Btn(2), joy1Btn(3) || joy2Btn(3));

	  // Output
	  motor[dtl.id] = -dtr.output;
	  motor[dtr.id] = -dtl.output;
	  motor[fr.id] = fr.output;


	  if(joy1Btn(9) || joy2Btn(9)) {
	  	hardReset();
		}
  }
}

int scaleJoyValue(float p_joy_val) {
	// If the joystick value is the within the deadzone, do not move motors
	if(abs(p_joy_val) < 3) {
		return 0;
	} else {
		int direction = p_joy_val / abs(p_joy_val);
		return  direction * (p_joy_val*p_joy_val) / (127*127) * 80;
	}
}

void initializeRobot() {
  motor[dt_left] = 0;
  motor[dt_right] = 0;
  motor[flag] = 0;
  motor[hang] = 0;

  servo[auto_block] = 255;
  servoChangeRate[auto_block] = 0;

  servo[bucket] = 255;
  servoChangeRate[bucket] = 0;

  return;
}
