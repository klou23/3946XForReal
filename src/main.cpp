/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kevin Lou                                                 */
/*    Created:      Fri Nov 9 2019                                            */
/*    Description:  3946X Code OFFICAL                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"

using namespace vex;

/*----------------------------------------------------------------------------*/
/*           Global Instances of motors, device, and competition             */
/*----------------------------------------------------------------------------*/

// A global instance of competition
competition Competition;

//global instances of motors and other devices here
//vex::motor leftFrontDrive(vex::PORT12, vex::gearSetting::ratio18_1, false);
//vex::motor rightFrontDrive(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor leftDrive(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor lift1(vex::PORT3, vex::gearSetting::ratio36_1, false);
vex::motor lift2(vex::PORT4, vex::gearSetting::ratio36_1, true);
vex::motor lift3(vex::PORT5, vex::gearSetting::ratio36_1, true);
vex::motor leftRoller(vex::PORT6, vex::gearSetting::ratio36_1, false);
vex::motor rightRoller(vex::PORT7, vex::gearSetting::ratio36_1, false);
vex::motor shifter(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::controller controller1 = vex::controller();

/*---------------------------------------------------------------------------*/
/*                             Variables, Arrays                             */
/*---------------------------------------------------------------------------*/

/*
  Auton List:
  1. Blue 4 cube line (4 points)
  2. Red 4 cube line (4 points)
  3. Blue 1 cube and 4 cube stack (5 points)
  4. Red 1 cube and 4 cube stack (5 points)
*/

int Auton = 1;

motor motorArray[8] = {leftDrive, rightDrive, lift1, lift2, lift3, leftRoller, rightRoller, shifter};
std::string motorNames[8] = {"Left Drive", "Right Drive", "Lift 1", "Lift 2", "Lift 3", "Left Roller", "Right Roller", "Shifter"};

double rightDriveVel = 0;
double leftDriveVel = 0;

/*---------------------------------------------------------------------------*/
/*                                Functions                                  */
/*---------------------------------------------------------------------------*/

void controllerDrive(void){
  rightDriveVel = controller1.Axis2.value() * 0.8;
  leftDriveVel = controller1.Axis3.value() * 0.8;
  rightDrive.spin(directionType::fwd, rightDriveVel, velocityUnits::pct);
  leftDrive.spin(directionType::fwd, leftDriveVel, velocityUnits::pct);
}

void liftUp(double speed){
  lift1.spin(directionType::fwd, speed, velocityUnits::pct);
  lift2.spin(directionType::fwd, speed, velocityUnits::pct);
  lift3.spin(directionType::fwd, speed, velocityUnits::pct);
}

void liftDown(double speed){
  lift1.spin(directionType::rev, speed, velocityUnits::pct);
  lift2.spin(directionType::rev, speed, velocityUnits::pct);
  lift3.spin(directionType::rev, speed, velocityUnits::pct);
}

void liftStop(void){
  lift1.stop(hold);
  lift2.stop(hold);
  lift3.stop(hold);
}

void liftUpFor(double time, double speed){
  liftUp(speed);
  wait(time, msec);
  liftStop();
}

void liftDownFor(double time, double speed){
  liftDown(speed);
  wait(time, msec);
  liftStop();
}

void drive(double time, double speed){
  leftDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  rightDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  wait(time, msec);
  leftDrive.stop();
  rightDrive.stop();
}

void turnCW(double time, double speed){
  leftDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  rightDrive.spin(directionType::rev, speed, velocityUnits::pct);
  wait(time, msec);
  leftDrive.stop();
  rightDrive.stop();
}

void turnCCW(double time, double speed){
  leftDrive.spin(directionType::rev, speed, velocityUnits::pct);
  rightDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  wait(time, msec);
  leftDrive.stop();
  rightDrive.stop();
}

double testTemp(motor testMotor){
  return testMotor.temperature();
}

bool isOverTemp(motor motorName, int temp){
  bool returnValue = false;
  if(motorName.temperature(temperatureUnits::fahrenheit) > temp){
    returnValue = true;
  }
  return returnValue;
}

bool allMotorsOverTemp(motor motors[8], int temp){
  bool returnValue = false;
  for(int i = 0; i < 8; i++){
    if(isOverTemp(motors[i], temp)){
      returnValue = true;
    }
  }
  return returnValue;
}

int circleColor(void){
  int returnValue;
  if(allMotorsOverTemp(motorArray, 150)){
    returnValue = 0;
  }else if(allMotorsOverTemp(motorArray, 125)){
    returnValue = 30;
  }else if(allMotorsOverTemp(motorArray, 100)){
    returnValue = 60;
  }else {
    returnValue = 90;
  }
  return returnValue;
}

void rollerIntake(double speed){
  leftRoller.spin(directionType::fwd, speed, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, speed, velocityUnits::pct);
}

void rollerExtake(double speed){
  leftRoller.spin(directionType::fwd, speed, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, speed, velocityUnits::pct);
}

void rollerStop(void){
  leftRoller.stop();
  rightRoller.stop();
}

void shifterUp(void){
  shifter.rotateTo(157, rotationUnits::deg, 15, velocityUnits::pct, true);
  shifter.stop(hold);
}

void shifterDown(void){
  shifter.rotateTo(0, rotationUnits::deg, 50, velocityUnits::pct, true);
  shifter.stop();
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  if(Auton == 1){
    rollerIntake(80);
    drive(1500, 50);
    turnCCW(500, 50);
    drive(1000,50);
    shifterUp();
    drive(250, -50);
    shifterDown();
  }else if(Auton == 2){
    rollerIntake(80);
    drive(1500, 50);
    turnCW(500, 50);
    drive(1000,50);
    shifterUp();
    drive(250, -50);
    shifterDown();
  }else if(Auton == 3){
    rollerIntake(80);
    drive(1000, 50);
    liftUpFor(800, 80);
    drive(500, 50);
    liftDownFor(1000, 50);
    turnCW(500, 50);
    drive(1000, 50);
    shifterUp();
    drive(250, -50);
    shifterDown();
  }else if(Auton == 4){
    rollerIntake(80);
    drive(1000, 50);
    liftUpFor(800, 80);
    drive(500, 50);
    liftDownFor(1000, 50);
    turnCCW(500, 50);
    drive(1000, 50);
    shifterUp();
    drive(250, -50);
    shifterDown();
  }
}



/*---------------------------------------------------------------------------*/
/*                            User Control Task                              */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // main execution loop for the user control program.
  while (1) {
    //drive
    controllerDrive();

    //lift
    if(controller1.ButtonR1.pressing()){
      liftUp(100);
    }else if(controller1.ButtonR2.pressing()){
      liftDown(100);
    }else{
      liftStop();
    }

    //roller
    if(controller1.ButtonL1.pressing()){
      rollerIntake(100);
    }else if(controller1.ButtonL2.pressing()){
      rollerExtake(100);
    }else{
      rollerStop();
    }

    //shifter
    if(controller1.ButtonUp.pressing()){
      shifterUp();
    }else if(controller1.ButtonDown.pressing()){
      shifterDown();
    }

    //motor temp stuff
    Brain.Screen.drawCircle(212, 60, 50, circleColor());
    for(int i = 0; i < 8 ; i++){
      if(isOverTemp(motorArray[i], 125)){
        Brain.Screen.setCursor(i+1, 1);
        Brain.Screen.print(motorNames[i].c_str());
      }
    }

    wait(20, msec); // Sleep the task to save resources
  }
}

/*---------------------------------------------------------------------------*/
/*                                  Main                                     */
/*---------------------------------------------------------------------------*/

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
