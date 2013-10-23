#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop, reversed)
// Drivetrain configuration - Direct drive dts built by Conner's previous students

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

const int DT_POWER_CAP = 80;
const int DRIVE_TIME = 3040;
// Moves forward 6 feet on a direct drive drivetrain.

void initializeRobot()
{
	motor[dt_left] = 0;
	motor[dt_right] = 0;

  return;
}

task main()
{
  initializeRobot();

  //waitForStart(); // Wait for the beginning of autonomous phase.

  motor[dt_left] = DT_POWER_CAP;
  motor[dt_right] = DT_POWER_CAP;

  wait1Msec(DRIVE_TIME);

  motor[dt_left] = 0;
  motor[dt_right] = 0;

  while (true)
  {}
}
