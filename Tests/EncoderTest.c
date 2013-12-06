#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop, reversed)

const int TICKS_PER_ROTATION = 1440;
const int WHEEL_CIRCUM = 3;

const int target_inches = 24;
const int motor_speed = 60;

void initializeRobot () {
	motor[dt_left] = 0;
	motor[dt_right] = 0;

	nMotorEncoder[dt_left] = 0;
}

int inchesToEncoder (float p_inches) {
	return p_inches * TICKS_PER_ROTATION / WHEEL_CIRCUM;
}

task main () {
	initializeRobot();

	motor[dt_left] = motor_speed;
	motor[dt_right] = motor_speed;

	while(nMotorEncoder[dt_left] < inchesToEncoder(target_inches)) {
		writeDebugStreamLine("%d", nMotorEncoder[dt_left]);
	}

	motor[dt_left] = 0;
	motor[dt_right] = 0;
}
