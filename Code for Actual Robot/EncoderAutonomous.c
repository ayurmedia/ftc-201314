#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ir_seeker,      sensorHiTechnicIRSeeker600)
#pragma config(Motor,  mtr_S1_C1_1,     dt_left,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     dt_right,      tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    auto_block,           tServoStandard)


const static int IR_SEEKING_VEL = 30;
const static int TOTAL_BRIDGE_DIST = 20;
const static int SERVO_CHANGE_RATE = 20;
const static int ARM_EXTENDED_POS = 90;
const static int SERVO_STOP_TIME = 1000;
const static int ARM_RETRIEVED_POS = 0;
const static int MAX_VEL = 80;
const static int CENTER_BRIDGE_DIST = 100;
const static int RAMP_DIST = 500;


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

void moveServoTo(short a_name, int a_position) {
  if(ServoValue[a_name] < a_position) {
    servo[a_name] = ServoValue[a_name] + SERVO_CHANGE_RATE;
  } else {
    servo[a_name] = ServoValue[a_name] - SERVO_CHANGE_RATE;
  }
}

int inchesToEncoder(float a_inches) {
  const static int  ticksPerRotation = 1350;
  const static float wheelCircumference = 3 * PI;

  return a_inches * ticksPerRotation / wheelCircumference;
}

bool turnRight() {
	const static int robotDiameter = 18;
	const static int turnDist = robotDiameter * PI / 4;
	const static int turnVel = 80;

	motor[dt_left] = -turnVel;
	motor[dt_right] = turnVel;
	while(abs(nMotorEncoder[dt_left]) < inchesToEncoder(turnDist)) {
    writeDebugStream("turnRight: %d", nMotorEncoder[dt_left]);
  }
	moveDT(0);

  return false;
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
        	wait10Msec(50);
          moveDT(0);
          m_state++;
        }
      break;

      // Moves auto_block servo to a certain position
      case kDispenseBlock:
      	writeDebugStream("servoValue: %d", ServoValue[auto_block]);
        if(ServoValue[auto_block] <= ARM_EXTENDED_POS) {
          moveServoTo(auto_block, ARM_EXTENDED_POS);
        } else {
        	wait1Msec(SERVO_STOP_TIME);
          m_state++;
        }
      break;

      // Pulls "arm" back to orgiinal position after dispensing the autonomous block
      case kRetrieveArm:
      	writeDebugStream("servoValue: %d", ServoValue[auto_block]);
        if(ServoValue[auto_block] != ARM_RETRIEVED_POS) {
          moveServoTo(auto_block, ARM_RETRIEVED_POS);
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
}

// Log - State machines may be well organized, but for what you are trying to do here, it may be overcomplicating the process.
//       This may work perfectly well for you, but if you start to encounter issues that you are having difficulty finding the reason for,
//       you may want to consider using a more straightforward structure for this. That being said, it looks good for what you said it needs to do.

  // "Special" RobotC thing where it needs a forever loop at the end of the autonomous program to wait for the end of the auonomous period
  for(;/*ever*/;) {}
}
