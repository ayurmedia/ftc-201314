#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     external_controller, sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     dt_back_left,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     dt_back_right, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     dt_front_left, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     dt_front_right, tmotorTetrix, openLoop)

// Include file to "handle" the Bluetooth messages.
#include "JoystickDriver.c"

// Constants
const int JOY_DEADZONE = 7;
const int MAX_DT_POWER = 80;
const float MAX_JOY_VAL = 127.0;

// Converts raw joystick values, which range from -128 to 127, into motor powers
int scaleJoyValue (float rawJoyVal) {
	// If the joystick value is the within the deadzone, do not move motors
	if(abs(rawJoyVal) < JOY_DEADZONE) {
		return 0;
	}

	// In all other cases, to scale the joystick value into motor value,
		// use an algorithm of function m(j) = (j^2)/(MJ^2) * MM,
		// where m = motor output, j = raw joystick value, MJ = MAX_JOY_VAL, and MM = MAX_DT_POWER.
	// HOWEVER, squaring the raw joystick value would eliminate the sign (+/-),
		// so extra precaution need be taken to preserve the direction.
	int direction = rawJoyVal / abs(rawJoyVal);
	return direction * (rawJoyVal*rawJoyVal) / (MAX_JOY_VAL*MAX_JOY_VAL) * MAX_DT_POWER;
}

// Set all drivetrain motors to 0
void initializeRobot() {
	motor[dt_back_left] = 0;
	motor[dt_front_left] = 0;
	motor[dt_back_right] = 0;
	motor[dt_front_right] = 0;
}

// Main loop; follows an IPO model
task main() {
	initializeRobot();
	// waitForStart();

	int dt_left_power = 0, dt_right_power = 0;

	for(;/*ever*/;) {
		// Get inputs
		// Updates joystick values
		getJoystickSettings(joystick);

		// Process inputs
		dt_left_power = scaleJoyValue(joystick.joy1_y1);
		dt_right_power = scaleJoyValue(joystick.joy1_y2);

		// Output to actuators
		motor[dt_back_left] = dt_left_power;
		motor[dt_front_left] = dt_left_power;
		motor[dt_back_right] = dt_right_power;
		motor[dt_front_right] = dt_right_power;

	}
}
