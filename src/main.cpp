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
vex::motor leftDrive(vex::PORT18, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor lift1(vex::PORT1, vex::gearSetting::ratio36_1, true);
vex::motor lift2(vex::PORT10, vex::gearSetting::ratio36_1, false);
vex::motor lift3(vex::PORT9, vex::gearSetting::ratio36_1, false);
vex::motor leftRoller(vex::PORT5, vex::gearSetting::ratio36_1, false);
vex::motor rightRoller(vex::PORT4, vex::gearSetting::ratio36_1, true);
vex::motor shifter(vex::PORT7, vex::gearSetting::ratio36_1, true);
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

  Driver skills:
  10. cool programming skills thing
*/

int Auton = 1;

motor motorArray[8] = {leftDrive, rightDrive, lift1, lift2, lift3, leftRoller, rightRoller, shifter};
std::string motorNames[8] = {"Left Drive", "Right Drive", "Left Lift", "Top Right Lift", "Bottom Right Lift", "Left Roller", "Right Roller", "Shifter"};

double rightDriveVel = 0;
double leftDriveVel = 0;
double distance;
double shifterSpeed;

/*---------------------------------------------------------------------------*/
/*                                Functions                                  */
/*---------------------------------------------------------------------------*/

void controllerDrive(void){
  rightDriveVel = controller1.Axis2.value() * 0.8 * 0.5;
  leftDriveVel = controller1.Axis3.value() * 0.8 * 0.5;
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
  leftRoller.spin(directionType::rev, speed, velocityUnits::pct);
  rightRoller.spin(directionType::rev, speed, velocityUnits::pct);
}

void rollerStop(void){
  leftRoller.stop(hold);
  rightRoller.stop(hold);
}

void shifterUp(void){
  
  // distance = 265;
  // while( distance >= 20){
  //   distance = 265 - abs((int)shifter.position(rotationUnits::deg));
  //   shifterSpeed = 100 * (distance/265);
  //   shifter.spin(directionType::fwd, shifterSpeed, velocityUnits::pct);
  //   Brain.Screen.clearScreen();
  //   Brain.Screen.setCursor(1, 1);
  //   Brain.Screen.print(distance);
  // }
  // shifter.stop();

  
  // shifter.rotateTo(-150, rotationUnits::deg, 100, velocityUnits::pct, false);
  // shifter.rotateTo(-370, rotationUnits::deg, 15, velocityUnits::pct);

  shifter.spin(directionType::rev, 15, velocityUnits::pct);
  
}

void shifterDown(void){
  shifter.rotateTo(0, rotationUnits::deg, 15, velocityUnits::pct, false);
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
  }else if(Auton == 10){
    rollerIntake(100);
    wait(250, msec);
    rollerExtake(100);
    wait(250, msec);
    rollerStop();
    liftUpFor(300, 100);
    drive(400, 100);
    liftDownFor(300, 100);
    rollerExtake(100);
    wait(250, msec);
    rollerStop();
    drive(400, -100);
    turnCCW(100, 100);
    rollerIntake(85);
    drive(750, 100);
    rollerStop();
    drive(50, -100);
    turnCCW(80, 100);
    liftUpFor(500, 100);
    drive(50, 100);
    rollerExtake(80);
    wait(250, msec);
    rollerStop();
    drive(50, -100);
    liftDownFor(500, 100);
    turnCW(80, 100);
    rollerIntake(85);
    drive(700, 100);
    rollerStop();
    turnCW(100, 100);
    drive(200, 100);
    shifterUp();
    drive(50, 100);
    turnCW(250, 100);
    rollerIntake(85);
    drive(500, 100);
    drive(50, -100);
    rollerStop();
    turnCCW(350, 100);
    liftUpFor(300, 100);
    drive(100, 100);
    rollerExtake(90);
    wait(250, msec);
    rollerStop();
    drive(50, -100);
    turnCCW(50, 100);
    drive(200, 100);
    turnCCW(200, 100);
    drive(600, 100);
    turnCCW(200, 100);
    rollerIntake(90);
    drive(1200, 100);
    rollerStop();
    turnCW(200, 100);
    drive(700, 100);
    shifterUp();
    drive(100, -100);
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
      liftDown(75);
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
     //shifterUp();
     shifter.spin(directionType::fwd, 15, velocityUnits::pct);
    }else if(controller1.ButtonDown.pressing()){
      //shifterDown();
      shifter.spin(directionType::rev, 15, velocityUnits::pct);
    }else{
      shifter.stop(hold);
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
