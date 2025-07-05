#include <SpeedyStepper.h>


//
// pin assignments
//

const int MOTOR_TL_STEP_PIN = 3;
const int MOTOR_TL_DIRECTION_PIN = 2;
const int MOTOR_ENABLE_TL= 26;

const int MOTOR_BL_STEP_PIN = 1;
const int MOTOR_BL_DIRECTION_PIN = 0;
const int MOTOR_ENABLE_BL = 14;

const int MOTOR_TR_STEP_PIN = 15;
const int MOTOR_TR_DIRECTION_PIN = 21;


const int MOTOR_BR_STEP_PIN = 22;
const int MOTOR_BR_DIRECTION_PIN = 23;


typedef struct {
    double x;
    double y;
} PointCartisian;

typedef struct {
    double r;
    double theta;
} PointPolar;

typedef struct{
    PointCartisian TL;
    PointCartisian TR;
    PointCartisian BL;
    PointCartisian BR;
} RobotWheels;

typedef struct{
    double x;
    double y;
    double theta;
} Input;

PointCartisian TL = {-5,5};
    PointCartisian TR = {5,5};
    PointCartisian BL = {-5,-5};
    PointCartisian BR = {5,-5};

  int stepsPerRevolution = 200*16;
    int TLcurrSteps = 0;
    int TRcurrSteps = 0;
    int BLcurrSteps = 0;
    int BRcurrSteps = 0;

PointCartisian rotate(PointCartisian wheel, double theta);
PointCartisian toCartesian(PointPolar p);
PointPolar toPolar(PointCartisian p);
PointCartisian translate(PointCartisian wheel, PointCartisian input);
RobotWheels moveRobot(RobotWheels robot, Input move);
PointCartisian convToUnitVector(PointCartisian start, PointCartisian end);
int sign (double x);
int toSteps(double inches, double wheelDiameter,int stepsPerRevolution);
double totalmove(PointCartisian start, PointCartisian end);

//
// create two stepper motor objects, one for each motor
//
SpeedyStepper steppertl;
SpeedyStepper stepperbl;

SpeedyStepper steppertr;
SpeedyStepper stepperbr;


void setup() 
{
  //
  // setup the LED pin and enable print statements
  //
    
  Serial.begin(115200);



double input[3] = {10,0,0};
RobotWheels Robot = {TL, TR, BL, BR};
    Input move = {input[0], input[1], input[2]};
    Robot = moveRobot(Robot, move);

    PointCartisian TLvec = convToUnitVector(TL, Robot.TL);
    PointCartisian TRvec = convToUnitVector(TR, Robot.TR);
    PointCartisian BLvec = convToUnitVector(BL, Robot.BL);
    PointCartisian BRvec = convToUnitVector(BR, Robot.BR);

    double TLspeed = (TLvec.x - TLvec.y) / 2;
    double TRspeed = (TRvec.x + TRvec.y) / 2;
    double BLspeed = (BLvec.x + BLvec.y) / 2;
    double BRspeed = (BRvec.x - BRvec.y) / 2;
    
    Serial.print(TLspeed);
    Serial.print(BLspeed);
    Serial.print(TRspeed);
    Serial.print(BRspeed);

    int TLsteps = toSteps(totalmove(TL, Robot.TL), 2.5, stepsPerRevolution);
    int TRsteps = toSteps(totalmove(TR, Robot.TR), 2.5, stepsPerRevolution);
    int BLsteps = toSteps(totalmove(BL, Robot.BL), 2.5, stepsPerRevolution);
    int BRsteps = toSteps(totalmove(BR, Robot.BR), 2.5, stepsPerRevolution);
  //
  // connect and configure the stepper motors to their IO pins
  //
  steppertl.connectToPins(MOTOR_TL_STEP_PIN, MOTOR_TL_DIRECTION_PIN);
  stepperbl.connectToPins(MOTOR_BL_STEP_PIN, MOTOR_BL_DIRECTION_PIN);
  steppertr.connectToPins(MOTOR_TR_STEP_PIN, MOTOR_TR_DIRECTION_PIN);
  stepperbr.connectToPins(MOTOR_BR_STEP_PIN, MOTOR_BR_DIRECTION_PIN);
  pinMode(MOTOR_ENABLE_TL, OUTPUT);
  digitalWrite(MOTOR_ENABLE_TL,LOW);
  pinMode(MOTOR_ENABLE_BL, OUTPUT);
  digitalWrite(MOTOR_ENABLE_BL,LOW);

  

  steppertl.setSpeedInStepsPerSecond(16*TLspeed+1);
  steppertl.setAccelerationInStepsPerSecondPerSecond(1000*16);
  steppertl.setupRelativeMoveInSteps(TLsteps*16);
 
  stepperbl.setSpeedInStepsPerSecond(500*16*BLspeed+1);
  stepperbl.setAccelerationInStepsPerSecondPerSecond(1000*16);
  stepperbl.setupRelativeMoveInSteps(BLsteps*16);
  
  
 

  stepperbr.setSpeedInStepsPerSecond(500*16*BRspeed+1);
  stepperbr.setAccelerationInStepsPerSecondPerSecond(1000*16);
  stepperbr.setupRelativeMoveInSteps(BRsteps*16);
  

 
   steppertr.setSpeedInStepsPerSecond(500*16*TRspeed+1);
  steppertr.setAccelerationInStepsPerSecondPerSecond(1000*16);
  steppertr .setupRelativeMoveInSteps(TRsteps*16);
  
  

 
}


void loop() 
{
//   steppertl.setSpeedInStepsPerSecond(100*16);
//   steppertl.setAccelerationInStepsPerSecondPerSecond(100*16);
//   steppertl.setupRelativeMoveInSteps(200*16);

//   stepperbl.setSpeedInStepsPerSecond(100*16);
//   stepperbl.setAccelerationInStepsPerSecondPerSecond(100*16);
//   stepperbl.setupRelativeMoveInSteps(-200*16);


//   stepperbr.setSpeedInStepsPerSecond(100*16);
//   stepperbr.setAccelerationInStepsPerSecondPerSecond(100*16);
//   stepperbr.setupRelativeMoveInSteps(-200*16);

//   steppertr.setSpeedInStepsPerSecond(100*16);
//   steppertr.setAccelerationInStepsPerSecondPerSecond(100*16);
//   steppertr .setupRelativeMoveInSteps(-200*16);
  
 while((!steppertl.motionComplete()) || (!steppertr.motionComplete()) || (!stepperbl.motionComplete()) || (!stepperbr.motionComplete()))
  {
     
    steppertl.processMovement();
    steppertr.processMovement();
    stepperbr.processMovement();
    stepperbl.processMovement();
  }
}




//
// move both X & Y motors together in a coordinated way, such that they each 
// start and stop at the same time, even if one motor moves a greater distance
//
void moveXYWithCoordination(long stepstl, long stepstr, long stepsbl, long stepsbr, float 
                            speedInStepsPerSecond_tl, float speedInStepsPerSecond_tr,float speedInStepsPerSecond_bl,float speedInStepsPerSecond_br,
                            float accelerationInStepsPerSecondPerSecond_tl,float accelerationInStepsPerSecondPerSecond_tr,float accelerationInStepsPerSecondPerSecond_bl,float accelerationInStepsPerSecondPerSecond_br)
{
  


  

  
  //
  // setup the motion for the X motor
  //
  steppertl.setSpeedInStepsPerSecond(speedInStepsPerSecond_tl);
  steppertl.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_tl);
  steppertl.setupRelativeMoveInSteps(stepstl);


  //
  // setup the motion for the Y motor
  //
  steppertr.setSpeedInStepsPerSecond(speedInStepsPerSecond_tr);
  steppertr.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_tr);
  steppertr.setupRelativeMoveInSteps(stepstr);

  stepperbr.setSpeedInStepsPerSecond(speedInStepsPerSecond_br);
  stepperbr.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_br);
  stepperbr.setupRelativeMoveInSteps(stepsbr);

  stepperbl.setSpeedInStepsPerSecond(speedInStepsPerSecond_bl);
  stepperbl.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_bl);
  stepperbl.setupRelativeMoveInSteps(stepsbl);


  //
  // now execute the moves, looping until both motors have finished
  //
  while((!steppertl.motionComplete()) || (!steppertr.motionComplete()) || (!stepperbl.motionComplete()) || (!stepperbr.motionComplete()))
  {
    steppertl.processMovement();
    steppertr.processMovement();
    stepperbr.processMovement();
    stepperbl.processMovement();
  }
}

PointCartisian toCartesian(PointPolar p){
    double x = p.r * cos(p.theta * M_PI / 180.0);
    double y = p.r * sin(p.theta * M_PI / 180.0);
    PointCartisian Cartesian = {x, y};
    return Cartesian;
}

PointPolar toPolar(PointCartisian p){
    double r = sqrt(p.x * p.x + p.y * p.y);
    double theta = atan2(p.y, p.x) * 180.0 / M_PI;
    PointPolar Polar = {r, theta};
    return Polar;
}

PointCartisian rotate(PointCartisian wheel, double theta){
    PointPolar rotated = toPolar(wheel);
    rotated.theta += theta;
    PointCartisian newCoordinates = toCartesian(rotated);
    return newCoordinates;
}

PointCartisian translate(PointCartisian wheel, PointCartisian input){
    PointCartisian newCoordinates = {wheel.x + input.x, wheel.y + input.y};
    return newCoordinates;
}

RobotWheels moveRobot(RobotWheels robot, Input move){
    RobotWheels newRobot;
    newRobot.TL = rotate(robot.TL, move.theta);
    newRobot.TR = rotate(robot.TR, move.theta);
    newRobot.BL = rotate(robot.BL, move.theta);
    newRobot.BR = rotate(robot.BR, move.theta);
    newRobot.TL = translate(newRobot.TL, (PointCartisian){move.x, move.y});
    newRobot.TR = translate(newRobot.TR, (PointCartisian){move.x, move.y});
    newRobot.BL = translate(newRobot.BL, (PointCartisian){move.x, move.y});
    newRobot.BR = translate(newRobot.BR, (PointCartisian){move.x, move.y});
    return newRobot;
}

PointCartisian convToUnitVector(PointCartisian start, PointCartisian end){
    PointCartisian p = {end.x - start.x, end.y - start.y};
    // double direction_x = sign(end.x);
    // double direction_y = sign(end.y);
    if(p.x < 0.001 && p.x > -0.001 && p.y < 0.001 && p.y > -0.001){
        PointCartisian unitVector = {0, 0};
        return unitVector;
    }
        
    double magnitude = sqrt(p.x*p.x + p.y*p.y);
    if (magnitude == 0) {
        PointCartisian unitVector = {0, 0};
        return unitVector;
    }
    double unitX = (p.x/magnitude);
    double unitY = (p.y/magnitude);
    PointCartisian unitVector = {unitX, unitY};
    return unitVector;
}

int sign (double x){
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

int toSteps(double inches, double wheelDiameter,int stepsPerRevolution){
    double circumference = M_PI * wheelDiameter;
    return (inches / circumference) * stepsPerRevolution;
}

double totalmove(PointCartisian start, PointCartisian end){
    double x = end.x - start.x;
    double y = end.y - start.y;
    return sqrt(x*x + y*y);
}
