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
int shifterDown = 500;
int shifterUp = 2727;
float gyroKp = 1;
float gyroKd = 0.0;
bool gyroCalibrated = false;
float shifterKp = 0.055; 
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
float angleSub(float deg1, float deg2) {
  float dist = deg1 - deg2;
  return fmod((dist + 180), 360) - 180;
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
void driveDistRollers(float amount, float speed, float rollerDelay = 0) {
  rightDrive.startRotateFor(amount, rotationUnits::deg, speed,
                            velocityUnits::pct);
  leftDrive.startRotateFor(amount, rotationUnits::deg, speed,
                           velocityUnits::pct);
  wait(rollerDelay, timeUnits::msec);
  runRollers(100);
  while (!rightDrive.isDone() || !leftDrive.isDone()) {
  }
}

void controllerDrive(void) {
  if (!controller2.ButtonX.pressing()) {
    leftDriveSpeed = controller1.Axis3.value();
    rightDriveSpeed = controller1.Axis2.value();
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

void rollerStop() {
  leftRoller.stop(hold);
  rightRoller.stop(hold);
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
    shifter1.stop();
    shifter2.stop();
    /** 
    // Stack done
    shifter1.stop();
    shifter2.stop();
    driveDist(-400, 40);
    wait(600,msec);
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
  while (timeInZone < 8) {
    float speed = angleSub(amount, gyroscope.yaw()) * gyroKp -
                  fabs((gyroscope.yaw() - lastYaw)) * gyroKd;
    lastYaw = gyroscope.yaw();
    leftDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    rightDrive.spin(directionType::rev, speed, velocityUnits::pct);
    wait(10, timeUnits::msec);
    if (fabs(angleSub(gyroscope.yaw(), amount)) < 2)
      timeInZone++;
    else
      timeInZone = 0;
  }

  rightDrive.stop(hold);
  leftDrive.stop(hold);
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
  Brain.Screen.setCursor(2,26);
  Brain.Screen.print(currentAutonName.c_str());

  // Battery
  
  drawRectangle(0, 0, 240, 45, black, black);
  Brain.Screen.setCursor(2,2);
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
/*                              Autonomous Task */
/*---------------------------------------------------------------------------*/
void blueProtected() {

  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(10, msec);
    }
  }

  // flip out tray
  rollerExtake();
  wait(300, timeUnits::msec);
  rollerStop();

  // wait for ghost tray to fall
  wait(800, timeUnits::msec);

  // intake first cube
  runRollers(50);
  driveDist(1200, 100);
  rollerStop();

  // second cube
  gyroTurnTo(90);
  driveDistRollers(1000, 100, 500);
  rollerStop();

  // third cube
  gyroTurnTo(-10);
  driveDistRollers(1500, 100, 925);
  rollerStop();

  // go to zone
  driveDist(-1000, 100);
  gyroTurnTo(140);
  driveDist(940, 100);

  // place stack
  while (fabs(shifterUp - shifterPot.value(vex::analogUnits::range12bit)) >=
         30) {
    shifter1.spin(directionType::fwd, shifterStackSpeed(), velocityUnits::pct);
    shifter2.spin(directionType::fwd, shifterStackSpeed(), velocityUnits::pct);
  }
  driveDist(-1000, 100);
}
void autonomous(void) {
  /*
  Important stuff:
  lift code not implemented, so UPLOAD TO PROGRAM SLOT 2. CHANGE PROGRAM SLOT
  TO THE LEFT OF THE PROGRAM NAME AT THE TOP USE THIS PROGRAM FOR AUTON, USE
  THE OTHER ALREADY DOWNLOADED PROGRAM FOR DRIVER PRACTICE SHIFTER CODE NOT
  IMPLEMENTED
  */
  if (!gyroCalibrated) {
    gyroscope.calibrate();
    while (gyroscope.isCalibrating()) {
      wait(10, msec);
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    controllerDrive();
    if (controller1.ButtonL1.pressing()) {
      rollerIntake();
    } else if (controller1.ButtonL2.pressing()) {
      rollerExtake();
    } else {
      rollerStop();
    }

    if (controller1.ButtonUp.pressing()) {
      autoStack();
    } else if (controller1.ButtonDown.pressing()) {
      if(shifterPot.value(analogUnits::range12bit) > shifterDown){
        manualShifterDown();
      }
    } else if (controller1.ButtonB.pressing()) {
      autonShifterDown();
    }

    else {
      shifterHold();
    }

    if (controller1.ButtonR1.pressing()) {
      lift(100);
    } else if (controller1.ButtonR2.pressing()) {
      lift(-100);
    } else {
      liftHold();
    }
    if (controller1.ButtonA.pressing()) {
      gyroTurnTo(90);
      driveDist(1000, 100);
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