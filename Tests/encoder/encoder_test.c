#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
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

task main () {
	const static short mot = dt_left;

	clearDebugStream();
	motor[mot] = 0;
	nMotorEncoder[mot] = 0;
	time1[T1] = 0;

	motor[mot] = 50;
	while(time1[T1]<=20000) {
		writeDebugStreamLine("%d, %d", time1[T1], nMotorEncoder[mot]);
	}
	motor[mot] = 0;
}
