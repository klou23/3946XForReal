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

// A global instance of competition
competition Competition;

// global instances of motors and other devices here
vex::motor leftDrive(vex::PORT18, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor rightLift(vex::PORT17, vex::gearSetting::ratio36_1, true);
vex::motor leftLift(vex::PORT16, vex::gearSetting::ratio36_1, true);
vex::motor leftRoller(vex::PORT1, vex::gearSetting::ratio36_1, false);
vex::motor rightRoller(vex::PORT3, vex::gearSetting::ratio36_1, true);
vex::motor shifter1(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::motor shifter2(vex::PORT11, vex::gearSetting::ratio36_1, true);
vex::controller controller1 = vex::controller(controllerType::primary);
vex::controller controller2 = vex::controller(controllerType::partner);
vex::inertial gyroscope(vex::PORT12);

//global variable
double leftDriveSpeed;
double rightDriveSpeed;

/*---------------------------------------------------------------------------*/
/*                                 Functions                                 */
/*---------------------------------------------------------------------------*/

void controllerDrive(void){
  if(!controller2.ButtonX.pressing()){
    leftDriveSpeed = ((controller1.Axis3.value()/100)*(controller1.Axis3.value()/100)) * 100;
    rightDriveSpeed = ((controller1.Axis2.value()/100)*(controller1.Axis2.value()/100)) * 100;
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
  leftRoller.spin(directionType::fwd, 80, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, 80, velocityUnits::pct);
}

void rollerStop(){
  leftRoller.stop(hold);
  rightRoller.stop(hold);
}

void manualShifterUp(void){
  shifter1.spin(directionType::fwd, 40, velocityUnits::pct);
  shifter2.spin(directionType::fwd, 40, velocityUnits::pct);
}

void manualShifterDown(void){
  shifter1.spin(directionType::rev, 40, velocityUnits::pct);
  shifter2.spin(directionType::rev, 40, velocityUnits::pct);
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

void driveFwd(int amount, double speed){
  Drivetrain.setDriveVelocity(speed, percent);
  Drivetrain.driveFor(forward, amount, inches);
}

void driveRev(int amount, double speed){
  Drivetrain.setDriveVelocity(speed, percent);
  Drivetrain.driveFor(reverse, amount, inches);
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
