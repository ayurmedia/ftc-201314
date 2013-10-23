// Light sensor follows blue/red line until IR seeker centers on the IR beacon.

#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     external_controller, sensorNone)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     dt_back_left,  tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C1_2,     dt_front_left, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     dt_back_right, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     dt_front_right, tmotorTetrix, openLoop)
// Configuration - wood drivetrain built by dt team

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

const int DT_POWER = 50;

void setDTPower(int power) {
	motor[dt_back_left] = power;
	motor[dt_front_left] = power;
	motor[dt_back_right] = power;
	motor[dt_front_right] = power;
}

void initializeRobot() {
	setDTPower(0);
  return;
}

task main() {
  initializeRobot();

  //waitForStart(); // Wait for the beginning of autonomous phase.

	while (SensorValue(ir_seeker)!=5){
			setDTPower(DT_POWER);
	}
	setDTPower(0);

  while (true)
  {}
}
