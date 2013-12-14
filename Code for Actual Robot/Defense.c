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

short ENCODER = dt_right;

int MAX_POWER = 80;
int MOVE_DIST_1 = 40;
int TURN_DEGREES_1 = 60;
int MOVE_DIST_2 = 40;
int TURN_DEGREES_2 = 60;
int IR_VEL = 30;
int LAST_BUCKET_DIST = 40;
int TURN_DEGREES_3 = 90;


void initializeRobot() {
	// Make sure all motors are not moving before waitForStart()
	motor[dt_left] = 0;
	motor[dt_right] = 0;
	nMotorEncoder[ENCODER] = 0;

	// Set default speed for the auto_block servo
	servoChangeRate[auto_block] = 0;
	// Set the initial position for the auto_block servo
	servo[auto_block] = 0;

	return;
}

void moveDT(int a_speed) {
	motor[dt_right] = a_speed;
	motor[dt_left] = a_speed;

}

int inchesToEncoder(float p_inches) {
	// Calibrated December 11, 2013
	return 140.74 * p_inches - 254.72;
	/* return p_inches * TICKS_PER_ROTATION / WHEEL_CIRCUM; */
}

void turnDegrees(int p_degrees) {
	const int ROBOT_DIAMETER = 20;
	const int TURN_VEL = -60;

	const int turn_dist = ROBOT_DIAMETER * PI * p_degrees / 360;
	int direction = p_degrees / abs(p_degrees);

	nMotorEncoder[ENCODER] = 0;

	do {
		motor[dt_left] = -TURN_VEL * direction;
		motor[dt_right] = TURN_VEL * direction;
		writeDebugStream("turnDegrees: %d, %d", nMotorEncoder[ENCODER], inchesToEncoder(turn_dist));
	} while(abs(nMotorEncoder[ENCODER]) < inchesToEncoder(abs(turn_dist)));

	moveDT(0);
}

void moveInches(int p_inches) {
	const int encoder_dist = inchesToEncoder(p_inches);

	nMotorEncoder[ENCODER] = 0;
	do {
		moveDT(MAX_POWER);
	} while(abs(nMotorEncoder[ENCODER]) < abs(encoder_dist));
}

task main() {
	initializeRobot();
	//waitForStart(); // Wait for the beginning of autonomous phase.

	// Move forward
	moveInches(MOVE_DIST_1);

	// Turn to be parallel to wall
	turnDegrees(TURN_DEGREES_1);

	// Move foward
	moveInches(MOVE_DIST_2);

	// Turn to be parallel to the bridge
	turnDegrees(TURN_DEGREES_2);

	// Move to find IR or until the last bucket
	nMotorEncoder[ENCODER] = 0;
	do {
		moveDT(IR_VEL);
	} while(SensorValue[ir_seeker] != 5 && nMotorEncoder[ENCODER] < inchesToEncoder(LAST_BUCKET_DIST));
	moveDT(0);

	// Turn 90 degrees
	turnDegrees(TURN_DEGREES_3);

	MOVEDT(0);


	// "Special" RobotC thing where it needs a forever loop at the end of the autonomous program to wait for the end of the auonomous period
		for(;/*ever*/;) {}
}