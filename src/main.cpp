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
vex::motor leftDrive(vex::PORT18, vex::gearSetting::ratio18_1, false);
vex::motor rightDrive(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor lift1(vex::PORT17, vex::gearSetting::ratio36_1, true);
vex::motor lift2(vex::PORT10, vex::gearSetting::ratio36_1, false);
vex::motor lift3(vex::PORT6, vex::gearSetting::ratio36_1, false);
vex::motor leftRoller(vex::PORT2, vex::gearSetting::ratio36_1, false);
vex::motor rightRoller(vex::PORT3, vex::gearSetting::ratio36_1, true);
vex::motor shifter(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::controller controller1 = vex::controller();

/*---------------------------------------------------------------------------*/
/*                             Variables, Arrays                             */
/*---------------------------------------------------------------------------*/

/*
  Auton List:
  1. Blue 4 cube line (5 points)
  2. Red 4 cube line (5 points)
  3. Blue 1 cube and 4 cube stack (6 points)
  4. Red 1 cube and 4 cube stack (6 points)

  Driver skills:
  10. cool programming skills thing
*/

int Auton = 2;

motor motorArray[8] = {leftDrive, rightDrive, lift1, lift2, lift3, leftRoller, rightRoller, shifter};
std::string motorNames[8] = {"Left Drive", "Right Drive", "Left Lift", "Bottom Right Lift", "Top Right Lift", "Left Roller", "Right Roller", "Shifter"};

double rightDriveVel = 0;
double leftDriveVel = 0;
double distance;
double shifterSpeed;
bool shifterGoingUp = false;
double kp = 0.15;
double ki = 0.05;
int totalError = 0;
int error;
int target;
int p;
int i;

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

void turnMotor(double degrees, double time, double speed, motor spinningMotor){
  bool goingFwd;
  if(speed < 0){
    goingFwd = false;
  }else{
    goingFwd = true;
  }
  int revolutions = 0;
  int lastPosition = 0;
  int degreesTurned = 0;
  int loops = 0;
  int currentPosition = 0;
  if(goingFwd){
    while(loops * 10 < time && degrees - degreesTurned > 0){
      spinningMotor.spin(directionType::fwd, speed, velocityUnits::pct);
      lastPosition = currentPosition;
      currentPosition = spinningMotor.position(rotationUnits::deg);
      if(lastPosition > currentPosition){
        revolutions++;
      }
      degreesTurned = currentPosition + revolutions * 360;
      wait(9, msec);
      loops ++;
    }
  }
  leftDrive.stop(hold);
  rightDrive.stop(hold);
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

void shifterDown(void){
  shifter.rotateTo(0, rotationUnits::deg, 15, velocityUnits::pct, false);
}

void shifterUp(void){
  
  distance = 270;
  while(distance >= 10){
    distance = abs(270 - abs((int)shifter.position(rotationUnits::deg)));
    shifterSpeed = (270 - (int) shifter.position(rotationUnits::deg)) * kp * 0.25;
    shifter.spin(directionType::rev, shifterSpeed, velocityUnits::pct);
  }
  shifter.stop(hold);
  rollerStop();

  // target = 200;
  // error = 200;
  // if(error > 10){
  //   error = target - (int) shifter.position(rotationUnits::deg);
  //   p = error * kp;
  //   totalError += error;
  //   i = totalError*ki;
  //   shifterSpeed = p+i;
  //   shifter.spin(directionType::rev, shifterSpeed, velocityUnits::pct);
  // }

}

void foldOut(void){
  rollerExtake(100);
  wait(500, msec);
  rollerIntake(100);
  wait(1000, msec);
  rollerExtake(100);
  wait(1000, msec);
  rollerStop();
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
  foldOut();
  liftDownFor(200, 100);
  if(Auton == 1){
    rollerIntake(80);
    drive(4500, 20);
    rollerStop();
    turnCCW(870, 100);
    liftUpFor(200, 50);
    drive(1800,50);
    rollerStop();
    rollerExtake(100);
    wait(200, msec);
    rollerStop();
    rollerExtake(5);
    shifterUp();
    rollerExtake(60);
    drive(400, -50);
    rollerStop();
    shifterDown();
  }else if(Auton == 2){
    rollerIntake(80);
    drive(4500, 20);
    rollerStop();
    turnCW(950, 100);
    liftUpFor(200, 50);
    drive(1800,50);
    rollerStop();
    rollerExtake(100);
    wait(200, msec);
    rollerStop();
    rollerExtake(5);
    shifterUp();
    rollerExtake(60);
    drive(400, -50);
    rollerStop();
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
    

    // if(shifterGoingUp){
    //   distance = abs(220 - abs((int)shifter.position(rotationUnits::deg)));
    //   if(distance >= 10){
    //     if(shifter.position(rotationUnits::deg) < 100){
    //       shifterSpeed = 100;
    //     }else {
    //       shifterSpeed = 10 * ((220 - distance)/220);
    //     }
    //     shifter.spin(directionType::rev, shifterSpeed, velocityUnits::pct);
    //     //rollerExtake(20);
    //   }else{
    //     shifter.stop(hold);
    //   }
    // }else{
    //   shifter.rotateTo(0, rotationUnits::deg, 15, velocityUnits::pct, false);
    // }

    error = target - (int) shifter.position(rotationUnits::deg);
    p = error * kp;
    totalError += error;
    i = totalError*ki;
    shifterSpeed = p+i;

    if(controller1.ButtonUp.pressing()){
      target = -300;
      totalError = 0;
      shifter.spin(directionType::fwd, shifterSpeed, velocityUnits::pct);
    }else if(controller1.ButtonDown.pressing()){
      target = 00;
      totalError = 0;
      shifter.spin(directionType::fwd, shifterSpeed, velocityUnits::pct);
    }else{
      shifter.stop(hold);
    }

    //shifter
    // if(controller1.ButtonRight.pressing()){
    //   shifter.spin(directionType::rev);
    // }else if(controller1.ButtonLeft.pressing()){
    //   shifter.spin(directionType::fwd);
    // }

    //motor temp stuff
    Brain.Screen.drawCircle(212, 60, 50, circleColor());
    for(int i = 0; i < 8 ; i++){
      if(isOverTemp(motorArray[i], 125)){
        Brain.Screen.setCursor(i+1, 1);
        Brain.Screen.print(motorNames[i].c_str());
      }
    }

    // Brain.Screen.setCursor(1, 10);
    // Brain.Screen.print("error: ");
    // Brain.Screen.setCursor(2, 10);
    // Brain.Screen.print(error);
    // Brain.Screen.setCursor(3, 10);
    // Brain.Screen.print("speed: ");
    // Brain.Screen.setCursor(4, 10);
    // Brain.Screen.print(shifterSpeed);
    // Brain.Screen.setCursor(5, 10);
    // Brain.Screen.print("target:");
    // Brain.Screen.setCursor(6, 10);
    // Brain.Screen.print(target);
    // Brain.Screen.setCursor(7, 10);
    // Brain.Screen.print("encoder:");
    // Brain.Screen.setCursor(8, 10);
    // Brain.Screen.print(shifter.position(rotationUnits::deg));

    if(controller1.ButtonX.pressing()){
      foldOut();
    }

    wait(5, msec); // Sleep the task to save resources
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
