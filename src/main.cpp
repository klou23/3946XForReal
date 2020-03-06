/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Authors:       Kevin Lou, Grant Fitez, Jack Hughes, Ethan Wang,         */
/*                   Frances Middleton-Davis, Lily Zook, Charlie Miller       */
/*    Created:      Wed Feb 26 2020                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>
#include <sstream>
#include <string>

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

// triport ThreeWirePort = vex::triport( vex::PORT22 );

// global variables, setup

std::string currentAutonName = "None";
double leftDriveSpeed;
double rightDriveSpeed;
int shifterDown = 400;
int shifterUp = 2700;
float gyroKp = 1.4;
float gyroKd = 0.0;
bool gyroCalibrated = false;
float shifterKp = 0.057;
int totalDistance = shifterDown - shifterUp;
int shifterPIDSpeed = 0;
int distanceError = 0;
int shifterMin = 12;
int currAutonID = -1; // 0 = Prog, 1 = Blue protected, 2 = Blue unprotected, 3 =
                      // Red Protected, 4 = Red unprotected, 5 = One cube push
int rgbHue = 0;
/*---------------------------------------------------------------------------*/
/*                                 Functions                                 */
/*---------------------------------------------------------------------------*/
void rollerStop() {
  leftRoller.stop(hold);
  rightRoller.stop(hold);
}
void driveHold() {

  rightDrive.stop(hold);
  leftDrive.stop(hold);
}
float angleSub(float deg1, float deg2) {
  float dist = deg1 - deg2;
  return fmod((dist + 180), 360) - 180;
}
float max(float a, float b) {
  if (a > b)
    return a;
  return b;
}
void rollerIntake() {
  leftRoller.spin(directionType::fwd, 100, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, 100, velocityUnits::pct);
}
void runRollers(int speed) {
  leftRoller.spin(directionType::fwd, speed, velocityUnits::pct);
  rightRoller.spin(directionType::fwd, speed, velocityUnits::pct);
}

void rollerExtake() {
  leftRoller.spin(directionType::rev, 80, velocityUnits::pct);
  rightRoller.spin(directionType::rev, 80, velocityUnits::pct);
}

void driveDist(float amount, float speed) {
  rightDrive.rotateFor(amount, rotationUnits::deg, speed, velocityUnits::pct,
                       false);
  leftDrive.rotateFor(amount, rotationUnits::deg, speed, velocityUnits::pct);
}
void clearDriveSensors(){
  leftDrive.resetPosition();
  rightDrive.resetPosition();
}
void driveDistRollers(float amount, float speed, float rollerDelay = 0) {
  clearDriveSensors();
  while((rightDrive.position(deg) + leftDrive.rotation(deg)) /2 < amount){
    rightDrive.spin(fwd,speed,pct);
    leftDrive.spin(fwd,speed,pct);
    if((rightDrive.position(deg) + leftDrive.rotation(deg)) /2 > rollerDelay){
      rollerIntake();
    }
  }
  driveHold();
}
void slowDrive(void) {
  //need to add sensitive zone
  if (!controller2.ButtonX.pressing()) {
    leftDriveSpeed = controller1.Axis3.value() * .5;
    rightDriveSpeed = controller1.Axis2.value() * .5;
    rightDrive.spin(directionType::fwd, rightDriveSpeed, velocityUnits::pct);
    leftDrive.spin(directionType::fwd, leftDriveSpeed, velocityUnits::pct);
  } else {
    leftDrive.stop(hold);
    rightDrive.stop(hold);
  }
}
void controllerDrive(void) {
  if (!controller2.ButtonX.pressing()) {
    leftDriveSpeed = (int) (pow(controller1.Axis3.value(), 2)/100);
    rightDriveSpeed = (int) (pow(controller1.Axis2.value(),2)/100);
    rightDrive.spin(directionType::fwd, rightDriveSpeed, velocityUnits::pct);
    leftDrive.spin(directionType::fwd, leftDriveSpeed, velocityUnits::pct);
  } else {
    leftDrive.stop(hold);
    rightDrive.stop(hold);
  }
}

void autonShifterDown(void) {
  while (shifterPot.value(analogUnits::range12bit) > shifterDown) {
    shifter1.spin(directionType::rev, 50, velocityUnits::pct);
    shifter2.spin(directionType::rev, 50, velocityUnits::pct);
  }
  shifter1.stop();
  shifter2.stop();
}

float shifterStackSpeed() {
  distanceError = shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  shifterPIDSpeed = distanceError * shifterKp;

  if (distanceError * shifterKp <= shifterMin) {
    shifterPIDSpeed = shifterMin;
  } else {
    shifterPIDSpeed = distanceError * shifterKp;
  }

  return shifterPIDSpeed;
}
void autoStack(void) {
  distanceError = shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  shifterPIDSpeed = shifterStackSpeed();
  if (abs(distanceError) <= 30) {

    // Stack done
    shifter1.stop();
    shifter2.stop();
    driveDist(-200, 40);
    wait(600, msec);
    /**
    while (shifterPot.value(analogUnits::range12bit) < shifterDown) {
    shifter1.spin(directionType::rev, 80, velocityUnits::pct);
    shifter2.spin(directionType::rev, 80, velocityUnits::pct);
    rollerIntake();
    }
    shifter1.stop();
    shifter2.stop();
    rollerStop();
    shifter1.stop();
    shifter2.stop();
    **/
  } else {

    shifter1.spin(directionType::fwd, shifterPIDSpeed, velocityUnits::pct);
    shifter2.spin(directionType::fwd, shifterPIDSpeed, velocityUnits::pct);
  }
}

void manualShifterDown(void) {
  float speed = 80;
  shifter1.spin(directionType::rev, speed, velocityUnits::pct);
  shifter2.spin(directionType::rev, speed, velocityUnits::pct);
}

void autonShifterUp(void) {
  shifter1.spin(directionType::fwd, 40, velocityUnits::pct);
  shifter2.spin(directionType::fwd, 40, velocityUnits::pct);
}

void shifterHold(void) {
  shifter1.stop(hold);
  shifter2.stop(hold);
}

void manualLiftUp(void) {
  leftLift.spin(directionType::fwd, 100, velocityUnits::pct);
  rightLift.spin(directionType::fwd, 100, velocityUnits::pct);
}

void manualLiftDown(void) {
  leftLift.spin(directionType::rev, 100, velocityUnits::pct);
  rightLift.spin(directionType::rev, 100, velocityUnits::pct);
}

void liftHold(void) {
  leftLift.stop(hold);
  rightLift.stop(hold);
}

void gyroTurnTo(double amount) {
  float lastYaw = 0;
  int timeInZone = 0;
  while (timeInZone < 1) {
    float speed = angleSub(amount, gyroscope.yaw()) * gyroKp -
                  fabs((gyroscope.yaw() - lastYaw)) * gyroKd;

    // if(speed>75 || speed<-75 )speed=75*fabs(speed)/speed;
    lastYaw = gyroscope.yaw();

    wait(2, timeUnits::msec);
    if (fabs(angleSub(gyroscope.yaw(), amount)) < 1) {
      timeInZone++;
      driveHold();
    } else {
      leftDrive.spin(directionType::fwd, speed, velocityUnits::pct);
      rightDrive.spin(directionType::rev, speed, velocityUnits::pct);
      timeInZone = 0;
    }
  }
  driveHold();
  wait(100, msec);
}

void lift(int speed) {
  leftLift.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
  rightLift.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
}

void liftTo(int height, int speed) {
  rightLift.rotateTo(height, rotationUnits::deg, speed, velocityUnits::pct,
                     false);
  leftLift.rotateTo(height, rotationUnits::deg, speed, velocityUnits::pct);
}

/*---------------------------------------------------------------------------*/
/*                                 GUI Code                                  */
/*---------------------------------------------------------------------------*/

void printToScreen(int x, int y, std::string str) {
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(x, y, str.c_str());
}

void printToScreenYellow(int x, int y, std::string str) {
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.printAt(x, y, str.c_str());
}

void printToScreenRed(int x, int y, std::string str) {
  Brain.Screen.setPenColor(red);
  Brain.Screen.printAt(x, y, str.c_str());
}
void drawRectangle(int x, int y, int width, int height, color fillColor,
                   color BorderColor) {
  Brain.Screen.setFillColor(fillColor);
  Brain.Screen.drawRectangle(x, y, width, height, BorderColor);
}

void drawTouch(void) {
  // Brain.Screen.setFillColor(orange);
  // Brain.Screen.drawCircle(Brain.Screen.xPosition(), Brain.Screen.yPosition(),
  //                         15, ClrGray);
}

void drawHeader(void) {
  // AutonBox
  drawRectangle(240, 0, 240, 45, orange, orange);
  Brain.Screen.setCursor(2, 26);
  Brain.Screen.print(currentAutonName.c_str());

  // Battery

  drawRectangle(0, 0, 240, 45, black, black);
  Brain.Screen.setCursor(2, 2);
  Brain.Screen.print("Battery Percentage: ");
  float brainBatt = (Brain.Battery.capacity());
  Brain.Screen.print(brainBatt);
}

void drawMainMenu(void) {
  drawHeader();

  // Debugger Button, top left
  drawRectangle(5, 50, 230, 90, black, black);
  printToScreen(10, 105, "Debugger");

  // Sensor Reset Button, bottom left
  drawRectangle(5, 145, 230, 90, black, black);
  printToScreen(10, 200, "Reset Sensors");

  // Auton Button, top right
  drawRectangle(245, 50, 230, 90, black, black);
  printToScreen(250, 105, "Select Autonomous");

  // Abort Button, bottom right
  drawRectangle(245, 145, 230, 90, black, black);
  printToScreen(250, 200, "Motor Temperatures");
}

void drawDebugger(void) {
  // This iteration's debugger variables: shifterKp, shifterPIDSpeed, shifterPot
  // in ticks, shifter error, drive moter encoder values, gyro values
  drawRectangle(0, 0, 480, 240, black, black);
  drawHeader();

  Brain.Screen.setCursor(4, 4);
  Brain.Screen.print("ShifterKp: ");
  Brain.Screen.print(shifterKp);
  Brain.Screen.setCursor(4, 25);
  Brain.Screen.print("  Autostack Speed: ");
  Brain.Screen.print(shifterPIDSpeed);
  Brain.Screen.setCursor(5, 4);
  Brain.Screen.print("Shifter Pot: ");
  Brain.Screen.print(shifterPot.value(vex::analogUnits::range12bit));
  Brain.Screen.setCursor(5, 25);
  Brain.Screen.print("  Autostack Error: ");
  Brain.Screen.print(distanceError);
  Brain.Screen.setCursor(6, 4);
  Brain.Screen.print("Left Drive Enc: ");
  Brain.Screen.print(leftDrive.position(rotationUnits::deg));
  Brain.Screen.setCursor(6, 25);
  Brain.Screen.print("  Right Drive Enc: ");
  Brain.Screen.print(rightDrive.position(rotationUnits::deg));
  Brain.Screen.setCursor(7, 4);
  Brain.Screen.print("Gyro X Axis: ");
  Brain.Screen.print(gyroscope.yaw());

  // Go back to main menu
  drawRectangle(245, 145, 230, 90, red, red);
  printToScreen(250, 180, "Main Menu");
}

void drawMainAutonScreen(void) {
  drawHeader();

  // Red Alliance Button, left
  drawRectangle(5, 50, 230, 130, red, red);
  printToScreen(10, 105, "Red Alliance");

  // Blue Alliance Button, right
  drawRectangle(245, 50, 230, 130, blue, blue);
  printToScreen(250, 105, "Blue Alliance");

  // Skills Button, lower center
  drawRectangle(5, 185, 475, 50, green, green);
  printToScreen(230, 200, "Skills");
}

void drawBlueAutonScreen(void) {
  drawHeader();

  // Main protected zone auton
  drawRectangle(5, 50, 230, 90, blue, blue);
  printToScreen(10, 85, "Blue-Protected Zone");

  // Main unprotected zone auton
  drawRectangle(5, 145, 230, 90, blue, blue);
  printToScreen(10, 180, "Blue-Unprotected Zone");

  // 1 cube push auton
  drawRectangle(245, 50, 230, 90, blue, blue);
  printToScreen(250, 85, "Blue-1 Cube Push Auton");

  // No auton
  drawRectangle(245, 145, 230, 90, blue, blue);
  printToScreen(250, 180, "Main Menu");
}

void drawRedAutonScreen(void) {
  drawHeader();

  // Main protected zone auton
  drawRectangle(5, 50, 230, 90, red, red);
  printToScreen(10, 85, "Red-Protected Zone");

  // Main unprotected zone auton
  drawRectangle(5, 145, 230, 90, red, red);
  printToScreen(10, 180, "Red-Unprotected Zone");

  // 1 cube push auton
  drawRectangle(245, 50, 230, 90, red, red);
  printToScreen(250, 85, "Red-1 Cube Push Auton");

  // No auton
  drawRectangle(245, 145, 230, 90, red, red);
  printToScreen(250, 180, "Main Menu");
}
void clearSensors(void) {
  // Clear the sensors
  gyroscope.setHeading(0, rotationUnits::deg);
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  leftLift.resetPosition();
  rightLift.resetPosition();
  leftRoller.resetPosition();
  rightRoller.resetPosition();
  shifter1.resetPosition();
  shifter2.resetPosition();
}
void drawMotorTempScreen() {
  drawRectangle(0, 0, 480, 240, black, black);
  drawHeader();

  Brain.Screen.setCursor(4, 4);
  Brain.Screen.print("Right Roller: ");
  Brain.Screen.print(rightRoller.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(4, 25);
  Brain.Screen.print("  Left Roller: ");
  Brain.Screen.print(leftRoller.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(5, 4);
  Brain.Screen.print("Shifter 1: ");
  Brain.Screen.print(shifter1.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(5, 25);
  Brain.Screen.print("  Shifter 2: ");
  Brain.Screen.print(shifter2.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(6, 4);
  Brain.Screen.print("Left Lift: ");
  Brain.Screen.print(leftLift.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(6, 25);
  Brain.Screen.print("  Right Lift: ");
  Brain.Screen.print(rightLift.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(7, 4);
  Brain.Screen.print("Left Drive: ");
  Brain.Screen.print(leftDrive.temperature(temperatureUnits::fahrenheit));
  Brain.Screen.setCursor(8, 4);
  Brain.Screen.print("Right Drive: ");
  Brain.Screen.print(rightDrive.temperature(temperatureUnits::fahrenheit));

  // Go back  button
  drawRectangle(245, 145, 230, 90, red, red);
  printToScreen(250, 180, "Main Menu");
}
/*---------------------------------------------------------------------------*/
/*                                 Pre-Auton                                 */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(10, msec);
    }
  }
  Brain.Screen.render(true, false);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  gyroscope.calibrate();
  while (gyroscope.isCalibrating()) {
    wait(10, msec);
  }
  gyroCalibrated = true;

  leftDrive.setBrake(coast);
  rightDrive.setBrake(coast);

  int currLocation = 0; // Main menu = 0; Debugger = 1, Autons = 2, Red Autons =
                        // 3, Blue Autons = 4, Motor Temperatures = 5
  while (true) {
    Brain.Screen.clearScreen();
    if (currLocation == 0) {
      drawMainMenu();
    } else if (currLocation == 1) {
      drawDebugger();
    } else if (currLocation == 2) {
      drawMainAutonScreen();
    } else if (currLocation == 3) {
      drawRedAutonScreen();
    } else if (currLocation == 4) {
      drawBlueAutonScreen();
    } else if (currLocation == 5) {
      drawMotorTempScreen();
    }
    Brain.Screen.render();
    // GUI Implementation
    switch (currAutonID) {
      switch (currAutonID) {
      case 0:
        currentAutonName = "Skills Routine";
        break;
      case 1:
        currentAutonName = "Blue Protected";
        break;
      case 2:
        currentAutonName = "Blue Unprotected";
        break;
      case 3:
        currentAutonName = "Red Protected";
        break;
      case 4:
        currentAutonName = "Red Unprotected";
        break;
      case 5:
        currentAutonName = "One Cube Push";
        break;
      }
    }
    if (Brain.Screen.pressing()) {
      while (Brain.Screen.pressing()) { // While being pressed, maintain image
                                        // and draw touch
        Brain.Screen.clearScreen();
        if (currLocation == 0) {
          drawMainMenu();
        } else if (currLocation == 1) {
          drawDebugger();
        } else if (currLocation == 2) {
          drawMainAutonScreen();
        } else if (currLocation == 3) {
          drawRedAutonScreen();
        } else if (currLocation == 4) {
          drawBlueAutonScreen();
        } else if (currLocation == 5) {
          drawMotorTempScreen();
        }
        drawTouch();
        Brain.Screen.render();
      }
      wait(50, timeUnits::msec); // Wait for user hand to get off of screen
      // All the rest of these if statements determine what todo with their
      // touch input

      if (currLocation == 0) {
        // IN MAIN MENU

        if (Brain.Screen.xPosition() >= 245 &&
            Brain.Screen.xPosition() <= 475) {
          // Right half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom right corner - Motor Temperatures
            currLocation = 5;
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top right button - Auton
            currLocation = 2;
          }
        } else if (Brain.Screen.xPosition() <= 240 &&
                   Brain.Screen.xPosition() >= 5) {
          // Left half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom left button - Reset Sensors
            clearSensors();
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top left button - Debugger
            currLocation = 1;
          }
        }
      } else if (currLocation == 1) {
        // In Debugger

        if (Brain.Screen.xPosition() >= 245 &&
            Brain.Screen.xPosition() <= 475 &&
            Brain.Screen.yPosition() >= 145 &&
            Brain.Screen.yPosition() <= 235) {
          // Pressed go back to main menu button
          currLocation = 0;
        }

      } else if (currLocation == 2) {
        // In Auton screen

        if (Brain.Screen.xPosition() >= 5 && Brain.Screen.xPosition() <= 235 &&
            Brain.Screen.yPosition() >= 50 && Brain.Screen.yPosition() <= 180) {
          // Left Button - Red
          currLocation = 3;
        } else if (Brain.Screen.xPosition() >= 245 &&
                   Brain.Screen.xPosition() <= 475 &&
                   Brain.Screen.yPosition() >= 50 &&
                   Brain.Screen.yPosition() <= 180) {
          // Right Button - Blue
          currLocation = 4;
        } else if (Brain.Screen.xPosition() >= 5 &&
                   Brain.Screen.xPosition() <= 475 &&
                   Brain.Screen.yPosition() >= 185 &&
                   Brain.Screen.xPosition() <= 235) {
          // Bottom button - skills
          currentAutonName = "Skills Routine";
          currLocation = 0;
          currAutonID = 0;
        }

      } else if (currLocation == 3) {
        // Red Autons

        if (Brain.Screen.xPosition() >= 245 &&
            Brain.Screen.xPosition() <= 475) {
          // Right half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom right corner - Main Menu
            currLocation = 0;
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top right button - One Cube
            currAutonID = 5;
            currentAutonName = "One Cube Push";
            currLocation = 0;
          }
        } else if (Brain.Screen.xPosition() <= 240 &&
                   Brain.Screen.xPosition() >= 5) {
          // Left half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom left button - Red Unprotected
            currentAutonName = "Red Unprotected";
            currAutonID = 4;
            currLocation = 0;
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top left button - Red Protected
            currentAutonName = "Red Protected";
            currAutonID = 3;
            currLocation = 0;
          }
        }
      } else if (currLocation == 4) {
        // Blue Autons
        if (Brain.Screen.xPosition() >= 245 &&
            Brain.Screen.xPosition() <= 475) {
          // Right half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom right corner - Main Menu
            currLocation = 0;
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top right button - One Cube
            currAutonID = 5;
            currentAutonName = "One Cube Push";
            currLocation = 0;
          }
        } else if (Brain.Screen.xPosition() <= 240 &&
                   Brain.Screen.xPosition() >= 5) {
          // Left half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom left button - Blue Unprotected
            currentAutonName = "Blue Unprotected";
            currAutonID = 2;
            currLocation = 0;
          } else if (Brain.Screen.yPosition() <= 140 &&
                     Brain.Screen.yPosition() >= 50) {
            // Top left button - Blue Protected
            currentAutonName = "Blue Protected";
            currAutonID = 1;
            currLocation = 0;
          }
        }
      } else if (currLocation == 5) {
        if (Brain.Screen.xPosition() >= 245 &&
            Brain.Screen.xPosition() <= 475) {
          // Right half of screen
          if (Brain.Screen.yPosition() >= 145 &&
              Brain.Screen.yPosition() <= 235) {
            // Bottom right corner - Main Menu
            currLocation = 0;
          }
        }
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Tasks */
/*---------------------------------------------------------------------------*/
void redProtected() {
  
  // flip out tray
  rollerExtake();
  wait(200, timeUnits::msec);
  rollerStop();
  shifter1.spin(fwd);
  shifter2.spin(fwd);
  wait(1000, msec);
  shifter1.stop();
  shifter2.stop();
  driveDist(100, 100);
  driveDist(-100, 100);

  // wait for ghost tray to fall
  wait(200, timeUnits::msec);

  // intake second cube
  driveDistRollers(800, 100, 200);
  wait(300,msec);
  rollerStop();

  // third cube
  driveDist(300, 100);
  gyroTurnTo(90);
  driveDistRollers(700, 100, 400);
  wait(300,msec);
  rollerStop();

  // fourth cube
  gyroTurnTo(93);
  driveDistRollers(850, 100, 300);
  wait(600, msec);
  rollerStop();

  // stack
  gyroTurnTo(-110);
  rollerExtake();
  wait(400,msec);
  rollerStop();
  driveDist(1700, 100);
  int autonDistanceError =
      shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  while (autonDistanceError >= 30) {
    int autonShifterPIDSpeed =
        shifterStackSpeed() * 1.7; // Faster because less cubes in auton
    autonDistanceError =
        shifterUp - shifterPot.value(vex::analogUnits::range12bit);
    shifter1.spin(fwd, autonShifterPIDSpeed, pct);
    shifter2.spin(fwd, autonShifterPIDSpeed, pct);
  }
  shifter1.stop();
  shifter2.stop();
  wait(200, msec);
  runRollers(-20);
  driveDist(-500, 100);
}

void blueProtected() {
  
  // flip out tray
  rollerExtake();
  wait(200, timeUnits::msec);
  rollerStop();
  shifter1.spin(fwd);
  shifter2.spin(fwd);
  wait(1000, msec);
  shifter1.stop();
  shifter2.stop();
  driveDist(100, 100);
  driveDist(-100, 100);

  // wait for ghost tray to fall
  wait(200, timeUnits::msec);

  // intake second cube
  driveDistRollers(800, 100, 200);
  wait(300,msec);
  rollerStop();

  // third cube
  driveDist(300, 100);
  gyroTurnTo(-90);
  driveDistRollers(700, 100, 400);
  wait(300,msec);
  rollerStop();

  // fourth cube
  gyroTurnTo(-93);
  driveDistRollers(850, 100, 300);
  wait(600, msec);
  rollerStop();

  // stack
  gyroTurnTo(-110);
  rollerExtake();
  wait(400,msec);
  rollerStop();
  driveDist(1700, 100);
  int autonDistanceError =
      shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  while (autonDistanceError >= 30) {
    int autonShifterPIDSpeed =
        shifterStackSpeed() * 1.7; // Faster because less cubes in auton
    autonDistanceError =
        shifterUp - shifterPot.value(vex::analogUnits::range12bit);
    shifter1.spin(fwd, autonShifterPIDSpeed, pct);
    shifter2.spin(fwd, autonShifterPIDSpeed, pct);
  }
  shifter1.stop();
  shifter2.stop();
  wait(200, msec);
  runRollers(-20);
  driveDist(-500, 100);
}

void blueUnprotected() {
  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(100, msec);
    }
  }
  // flip out tray
  rollerExtake();
  wait(300, timeUnits::msec);
  rollerStop();
  shifter1.spin(fwd);
  shifter2.spin(fwd);
  wait(1000, msec);
  shifter1.stop();
  shifter2.stop();
  driveDist(100, 100);
  driveDist(-100, 100);

  // wait for ghost tray to fall
  wait(200, timeUnits::msec);

  // Intake first line
  driveDistRollers(2300, 65, 1000);

  // Turn to drive back and align
  gyroTurnTo(45);

  // Drive back to intake second line
  driveDist(-3200, 100);

  // Turn to realign with line
  gyroTurnTo(0);

  // Intake second line
  driveDistRollers(3200, 65, 1000);

  // Drive back to get ready to turn and stack
  driveDist(-1000, 100);

  // turn to face goal zone
  gyroTurnTo(-135);

  // drive to goal zone
  driveDist(500, 100);

  // stack
  int autonDistanceError =
      shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  while (autonDistanceError >= 30) {
    int autonShifterPIDSpeed =
        shifterStackSpeed() * 1.7; // Faster because less cubes in auton
    autonDistanceError =
        shifterUp - shifterPot.value(vex::analogUnits::range12bit);
    shifter1.spin(fwd, autonShifterPIDSpeed, pct);
    shifter2.spin(fwd, autonShifterPIDSpeed, pct);
  }
  shifter1.stop();
  shifter2.stop();
  wait(200, msec);
  rollerExtake();
  driveDist(-500, 100);
}
void redUnprotectedLarge(){
  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(100, msec);
    }
  }
  // flip out tray
  shifter1.spin(fwd);
  shifter2.spin(fwd);
  rollerExtake();
  wait(300, timeUnits::msec);
  rollerStop();
  wait(700, msec);
  shifter1.stop();
  shifter2.stop();
  driveDist(100, 100);
  driveDist(-100, 100);

  // Intake first line
  driveDistRollers(1750, 65, 300);
  wait(400, msec);
  rollerStop();

  // Turn to back up towards large line
  gyroTurnTo(-33);
  driveDist(-1850, 100);
  // Turn and intake large line
  gyroTurnTo(0);
  driveDistRollers(1800, 57,100);
  rollerStop();

  //Turn towards goal zone
  gyroTurnTo(150);
  driveDist(1750, 100);
  int autonDistanceError =
      shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  while (autonDistanceError >= 150) {
    int autonShifterPIDSpeed =
        shifterStackSpeed() * 1.8; // Faster because less cubes in auton
    autonDistanceError =
        shifterUp - shifterPot.value(vex::analogUnits::range12bit);
    shifter1.spin(fwd, autonShifterPIDSpeed, pct);
    shifter2.spin(fwd, autonShifterPIDSpeed, pct);
  }
  shifter1.stop();
  shifter2.stop();
  wait(200, msec);
  rollerExtake();
  driveDist(-500, 100);
}
void redUnprotectedSafe() {
  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(100, msec);
    }
  }
  // flip out tray
  rollerExtake();
  wait(300, timeUnits::msec);
  rollerStop();
  shifter1.spin(fwd);
  shifter2.spin(fwd);
  wait(1000, msec);
  shifter1.stop();
  shifter2.stop();
  driveDist(100, 100);
  driveDist(-100, 100);

  // Intake first line
  driveDistRollers(1750, 65, 300);
  wait(400, msec);
  rollerStop();

  // Turn to face goblet cube 
  gyroTurnTo(-15);

  // Drive to intake goblet cube
  driveDistRollers(850,70,200);
  wait(200, msec);
  rollerStop();

  // drive back
  driveDist(-600, 100);

  // turn to face goal zone
  gyroTurnTo(147);

  // Drive to goal zone
  driveDist(1400, 100);

  int autonDistanceError =
      shifterUp - shifterPot.value(vex::analogUnits::range12bit);
  while (autonDistanceError >= 30) {
    int autonShifterPIDSpeed =
        shifterStackSpeed() * 1.7; // Faster because less cubes in auton
    autonDistanceError =
        shifterUp - shifterPot.value(vex::analogUnits::range12bit);
    shifter1.spin(fwd, autonShifterPIDSpeed, pct);
    shifter2.spin(fwd, autonShifterPIDSpeed, pct);
  }
  shifter1.stop();
  shifter2.stop();
  wait(200, msec);
  rollerExtake();
  driveDist(-500, 100);
}

void progSkills() {}

void oneCubePush() {}

void autonomous(void) {
  redUnprotectedLarge();

  // 0 = Prog, 1 = Blue protected, 2 = Blue unprotected, 3 =
  // Red Protected, 4 = Red unprotected, 5 = One cube push

  /**if (currAutonID == 0) {
    progSkills();
  } else if (currAutonID == 1) {
    blueProtected();
  } else if (currAutonID == 2) {
    blueUnprotected();
  } else if (currAutonID == 3) {
    redProtected();
  } else if (currAutonID == 4) {
    redUnprotected();
  } else if (currAutonID == 5) {
    oneCubePush();
  } else {
    wait(100, msec); // Do nothing
  }**/
}


/*---------------------------------------------------------------------------*/
/*                              User Control Task */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    if (controller2.ButtonL1.pressing()) {
      rollerIntake();
    } else if (controller2.ButtonL2.pressing()) {
      rollerExtake();
    } else if (controller1.ButtonL1.pressing()) {
      rollerIntake();
    } else if (controller1.ButtonL2.pressing()) {
      rollerExtake();
    } else {
      rollerStop();
    }
    if (controller2.ButtonR1.pressing()) {
      manualLiftUp();
    } else if (controller2.ButtonR2.pressing()) {
      manualLiftDown();
    } else if (controller1.ButtonR1.pressing()) {
      manualLiftUp();
    } else if (controller1.ButtonR2.pressing()) {
      manualLiftDown();
    } else {
      liftHold();
    }

    if (controller2.ButtonY.pressing()) {
      slowDrive();
    } else {
      controllerDrive();
    }

    if (controller1.ButtonUp.pressing() || controller2.ButtonUp.pressing()) {
      autoStack();
    } else if (controller1.ButtonDown.pressing() ||
               controller2.ButtonDown.pressing()) {
      if (shifterPot.value(analogUnits::range12bit) > shifterDown) {
        manualShifterDown();
      } else {
        shifterHold();
      }

    } else if (controller2.ButtonB.pressing()) {
      if (shifterPot.value(range12bit) < 920) {
        autoStack();
      } else if (shifterPot.value(range12bit) > 940) {
        manualShifterDown();
      }
    } else {
      shifterHold();
    }

    if (controller1.ButtonA.pressing()) {
      redUnprotectedLarge();
    } else {
      wait(100, msec); // Do nothing
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