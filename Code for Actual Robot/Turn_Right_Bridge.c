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
int MOVE_DIST_1 = 20;
int TURN_DEGREES = 80;
int MOVE_DIST_2 = 50;


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
	const int TURN_VEL = 60;

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
		writeDebugStreamLine("%d, %d", nMotorEncoder[ENCODER], encoder_dist);
	} while(abs(nMotorEncoder[ENCODER]) < abs(encoder_dist));
	moveDT(0);
}

task main() {
	initializeRobot();
	//waitForStart(); // Wait for the beginning of autonomous phase.

	// Move forward
	writeDebugStream("move 1: ");
	moveInches(MOVE_DIST_1);

	// Turn right
	writeDebugStream("turn: ");
	turnDegrees(TURN_DEGREES);

	// Move forward onto the bridge
	writeDebugStream("move 2: ");
	moveInches(MOVE_DIST_2);

	// "Special" RobotC thing where it needs a forever loop at the end of the autonomous program to wait for the end of the auonomous period
		for(;/*ever*/;) {}
}
