#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    auto_block,           tServoStandard)


const int IR_SEEKING_VEL = 30;
const int TOTAL_BRIDGE_DIST = 20;
const int ARM_EXTENDED_POS = 90;
const int SERVO_STOP_TIME = 1000;
const int ARM_RETRIEVED_POS = 0;
const int MAX_VEL = 80;
const int CENTER_BRIDGE_DIST = 100;
const int RAMP_DIST = 500;

const int ROBOT_DIAMETER = 20;
const float TURN_DIST = ROBOT_DIAMETER * PI / 4;
const int TURN_VEL = 60;

const int TICKS_PER_ROTATION = 1350;
const float WHEEL_CIRCUM = 3 * PI;


void initializeRobot() {
  // Make sure all motors are not moving before waitForStart()
  motor[dt_left] = 0;
  motor[dt_right] = 0;
  nMotorEncoder[dt_left] = 0;

  // Set default speed for the auto_block servo
  servoChangeRate[auto_block] = 0;
  // Set the initial position for the auto_block servo
  servo[auto_block] = ARM_RETRIEVED_POS;

  return;
}

void moveDT(int a_speed) {
	motor[dt_right] = a_speed;
  motor[dt_left] = a_speed;

}

int inchesToEncoder(float p_inches) {
  return p_inches * TICKS_PER_ROTATION / WHEEL_CIRCUM;
}

void turnRight() {
  do {
    motor[dt_left] = -TURN_VEL;
    motor[dt_right] = TURN_VEL;
    writeDebugStream("turnRight: %d", nMotorEncoder[dt_left]);
  } while(abs(nMotorEncoder[dt_left]) < inchesToEncoder(TURN_DIST));

	moveDT(0);
}

task main() {
  initializeRobot();
  //waitForStart(); // Wait for the beginning of autonomous phase.

  // Declares an enumeration defines all phases/states of the autonomous period
  typedef enum {
    kFindIR = 0,
    kDispenseBlock,
    kRetrieveArm,
    kPassBridge,
    kTurnRight1,
    kCenterBridge,
    kTurnRight2,
    kGetOnRamp,
    kFinishAuto
  } State;
  // Declares a State variable to iterate through all phases/states
  State m_state = kFindIR;

  // Main state machine
  while(m_state != kFinishAuto) {
  	writeDebugStream("State %d: ", m_state);
    switch(m_state) {
      // Move forward until IR seeker centers the IR beacon
      case kFindIR:
      	writeDebugStream("ir_seeker: %d", SensorValue[ir_seeker]);
        if(SensorValue[ir_seeker] != 5) {
          moveDT(IR_SEEKING_VEL);
        } else {
          moveDT(0);
          m_state++;
        }
      break;

      // Moves auto_block servo to a certain position
      case kDispenseBlock:
      	writeDebugStream("servoValue: %d", ServoValue[auto_block]);
        if(ServoValue[auto_block] != ARM_EXTENDED_POS) {
          servo[auto_block] = ARM_EXTENDED_POS;
        } else {
        	wait1Msec(SERVO_STOP_TIME);
          m_state++;
        }
      break;

      // Pulls "arm" back to orgiinal position after dispensing the autonomous block
      case kRetrieveArm:
      	writeDebugStream("servoValue: %d", ServoValue[auto_block]);
        if(ServoValue[auto_block] != ARM_RETRIEVED_POS) {
          servo[auto_block] = ARM_RETRIEVED_POS;
        } else {
          m_state++;
        }
      break;

      // Move straight forward a certain number of inches
      case kPassBridge:
      	writeDebugStream("kPassBridge: %d", nMotorEncoder[dt_left] );
        if(abs(nMotorEncoder[dt_left]) < inchesToEncoder(TOTAL_BRIDGE_DIST)) {
          moveDT(MAX_VEL);
        } else {
          moveDT(0);
          nMotorEncoder[dt_left] = 0;
          m_state++;
        }
      break;

      // Turn right 90 degrees
      case kTurnRight1:
        turnRight();
        nMotorEncoder[dt_left] = 0;
        m_state++;
      break;

      // Move straight forward a certain number of inches
      case kCenterBridge:
      	writeDebugStream("kCenterBridge: %d", nMotorEncoder[dt_left]);
        if(abs(nMotorEncoder[dt_left]) < inchesToEncoder(CENTER_BRIDGE_DIST)) {
          moveDT(MAX_VEL);
        } else {
          moveDT(0);
          nMotorEncoder[dt_left] = 0;
          m_state++;
        }
      break;

      // Turn right 90 degrees
      case kTurnRight2:
        turnRight();
        nMotorEncoder[dt_left] = 0;
        m_state++;
      break;

      // Move straight forward a certain number of encoder ticks
      case kGetOnRamp:
     		writeDebugStream("kGetOnRamp - %d", nMotorEncoder[dt_left]);
        if(abs(nMotorEncoder[dt_left]) < inchesToEncoder(RAMP_DIST)) {
          moveDT(MAX_VEL);
        } else {
          moveDT(0);
          nMotorEncoder[dt_left] = 0;
          m_state++;
        }
      break;

      // Indicates the end of the autonomous program
      default:
      break;

      writeDebugStreamLine("");
  }

  // "Special" RobotC thing where it needs a forever loop at the end of the autonomous program to wait for the end of the auonomous period
  for(;/*ever*/;) {}
}
