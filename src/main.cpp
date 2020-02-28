/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kevin Lou                                                       */
/*    Created:      Wed Feb 26 2020                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    18, 19, 12      
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
extern brain Brain;


// A global instance of competition
competition Competition;

// global instances of motors and other devices here
vex::motor leftDrive(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor rightLift(vex::PORT11, vex::gearSetting::ratio36_1, true);
vex::motor leftLift(vex::PORT16, vex::gearSetting::ratio36_1, false);
vex::motor leftRoller(vex::PORT20, vex::gearSetting::ratio36_1, false);
vex::motor rightRoller(vex::PORT13, vex::gearSetting::ratio36_1, true);
vex::motor shifter1(vex::PORT9, vex::gearSetting::ratio36_1, false);
vex::motor shifter2(vex::PORT3, vex::gearSetting::ratio36_1, true);
vex::controller controller1 = vex::controller(controllerType::primary);
vex::controller controller2 = vex::controller(controllerType::partner);
vex::inertial gyroscope(vex::PORT8);

vex::triport ThreeWirePort(vex::PORT22);
vex::pot shifterPot(ThreeWirePort.B);



//triport ThreeWirePort = vex::triport( vex::PORT22 );



//global variable
double leftDriveSpeed;
double rightDriveSpeed;
int shifterDown = 4095;
int shifterUp=1860;
float shifterKp=-0.043;//negative because sensor is reversed
int totalDistance = shifterDown-shifterUp;
/*---------------------------------------------------------------------------*/
/*                                 Functions                                 */
/*---------------------------------------------------------------------------*/

void controllerDrive(void){
  if(!controller2.ButtonX.pressing()){
    leftDriveSpeed = controller1.Axis3.value();
    rightDriveSpeed = controller1.Axis2.value();
    rightDrive.spin(directionType::fwd, rightDriveSpeed, velocityUnits::pct);
    leftDrive.spin(directionType::fwd, leftDriveSpeed, velocityUnits::pct);
  }else{
    leftDrive.stop(hold);
    rightDrive.stop(hold);
  }
}


void rollerIntake(){
  leftRoller.spin(directionType::fwd, 80, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, 80, velocityUnits::pct);
}

void rollerExtake(){
  leftRoller.spin(directionType::rev, 80, velocityUnits::pct);
  rightRoller.spin(directionType::rev, 80, velocityUnits::pct);
}

void rollerStop(){
  leftRoller.stop(hold);
  rightRoller.stop(hold);
}

void manualShifterUp(void){
  // int traveledDistance = shifterDown - shifterPot.angle();
  // float distanceError = totalDistance - traveledDistance;
  //float speed=shifterKp* distanceError;
  int distanceError = shifterUp-shifterPot.value(vex::analogUnits::range12bit);
  int speed = distanceError * shifterKp;
  shifter1.spin(directionType::fwd, speed, velocityUnits::pct);
  shifter2.spin(directionType::fwd, speed, velocityUnits::pct);
}

void manualShifterDown(void){
  float speed=80;
  shifter1.spin(directionType::rev, speed, velocityUnits::pct);
  shifter2.spin(directionType::rev, speed, velocityUnits::pct);
}

void autonShifterUp(void){
  shifter1.spin(directionType::fwd, 40, velocityUnits::pct);
  shifter2.spin(directionType::fwd, 40, velocityUnits::pct);
}

void autonShifterDown(void){

}

void shifterHold(void){
  shifter1.stop(hold);
  shifter2.stop(hold);
}

void manualLiftUp(void){
  leftLift.spin(directionType::fwd, 100, velocityUnits::pct);
  rightLift.spin(directionType::fwd, 100, velocityUnits::pct);
}

void manualLiftDown(void){
  leftLift.spin(directionType::rev, 100, velocityUnits::pct);
  rightLift.spin(directionType::rev, 100, velocityUnits::pct);
}

void liftHold(void){
  leftLift.stop(hold);
  rightLift.stop(hold);
}

void turnCW(double amount, double speed){
  gyroscope.setHeading(0, degrees);
  leftDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  rightDrive.spin(directionType::rev, speed, velocityUnits::pct);
  waitUntil(gyroscope.heading() >= amount);
  rightDrive.stop(hold);
  leftDrive.stop(hold);
}

void turnCCW(double amount, double speed){
  gyroscope.setHeading(0, degrees);
  leftDrive.spin(directionType::rev, speed, velocityUnits::pct);
  rightDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  waitUntil(gyroscope.heading() <= -amount);
  rightDrive.stop(hold);
  leftDrive.stop(hold);
}


void lift(int speed){
  leftLift.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
  rightLift.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
}
void driveDist(float amount, float speed){
  rightDrive.rotateFor(amount,rotationUnits::deg,speed,velocityUnits::pct, false);
  leftDrive.rotateFor(amount,rotationUnits::deg,speed,velocityUnits::pct);
}
void liftTo(int height, int speed){
  rightLift.rotateTo(-height, rotationUnits::deg, speed, velocityUnits::pct, false);
  leftLift.rotateTo(height, rotationUnits::deg, speed, velocityUnits::pct);
}
/*---------------------------------------------------------------------------*/
/*                                 Pre-Auton                                 */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  gyroscope.calibrate();
  while(gyroscope.isCalibrating()){
    wait(10, msec);
  }
  leftDrive.setBrake(coast);
  rightDrive.setBrake(coast);

}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  /*
  Important stuff:
  lift code not implemented, so UPLOAD TO PROGRAM SLOT 2. CHANGE PROGRAM SLOT TO THE LEFT OF THE PROGRAM NAME AT THE TOP
  USE THIS PROGRAM FOR AUTON, USE THE OTHER ALREADY DOWNLOADED PROGRAM FOR DRIVER PRACTICE
  SHIFTER CODE NOT IMPLEMENTED
  */

  //flip out tray
  rollerExtake();
  wait(300, timeUnits::msec);
  rollerStop();

  //wait for ghost tray to fall
  wait(800,timeUnits::msec);

  //intake first two cubes
  rollerIntake();
  driveDist(1000,100);

  //lift up and grab cube
  liftTo(214,100);
  driveFW
  rollerStop();




}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    controllerDrive();
    if(controller1.ButtonL1.pressing()){
      rollerIntake();
    }else if(controller1.ButtonL2.pressing()){
      rollerExtake();
    }else{
      rollerStop();
    }

    if(controller1.ButtonUp.pressing()){
      manualShifterUp();
    }else if(controller1.ButtonDown.pressing()){
      manualShifterDown();
    }else{
      shifterHold();
    }

    if(controller1.ButtonR1.pressing()){
      lift(100);
    }else if(controller1.ButtonR2.pressing()){
      lift(-100);
    }else{
      liftHold();
    }
  }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}