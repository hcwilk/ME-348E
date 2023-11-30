

#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

#define PI 3.141592653589

int leftFrontMotor; // COMMANDED MOTOR SPEEDS
int leftRearMotor;
int rightFrontMotor;
int rightRearMotor;

double leftMotorMax = 22.85; // **students should find this variable themselves**
double rightMotorMax = 21.44;

const int encoderLeftFrontPinA = 14;
const int encoderLeftFrontPinB = 15;

const int encoderLeftRearPinA = 16;
const int encoderLeftRearPinB = 17;

const int encoderRightFrontPinA = 23;
const int encoderRightFrontPinB = 22;

const int encoderRightRearPinA = 21;
const int encoderRightRearPinB = 20;



Encoder encoderLeftFront(encoderLeftFrontPinA,encoderLeftFrontPinB);
Encoder encoderLeftRear(encoderLeftRearPinA,encoderLeftRearPinB);
Encoder encoderRightFront(encoderRightFrontPinA, encoderRightFrontPinB);
Encoder encoderRightRear(encoderRightRearPinA,encoderRightRearPinB);


// From data sheet
double encoderResolution = 1632.67; // counts per rev
double d = 3.146; //wheel diameter in inches

int posLeftFrontCount = 0;
int posLeftRearCount = 0;
int posRightFrontCount = 0;
int posRightRearCount = 0;
int posLeftFrontCountLast = 0;
int posLeftRearCountLast = 0;
int posRightFrontCountLast = 0;
int posRightRearCountLast = 0;

double posLeftFrontRad = 0.0; // this will need to be converted to rad/sec
double posLeftRearRad = 0.0; // this will need to be converted to rad/sec
double posRightFrontRad = 0.0; // this will need to be converted to rad/sec
double posRightRearRad = 0.0; // this will need to be converted to rad/sec

double velLeftFront = 0; // this will be omegaLeft*d/2;
double velLeftRear = 0; // this will be omegaLeft*d/2;
double velRightFront = 0; // this will be omegaRight*d/2 will be in inches per sec;
double velRightRear = 0; // this will be omegaRight*d/2 will be in inches per sec;

double newVelLeftFront = 0; // this will be omegaLeft*d/2;
double newVelLeftRear = 0; // this will be omegaLeft*d/2;
double newVelRightFront = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelRightRear = 0; // this will be omegaRight*d/2 will be in inches per sec;

double rev_per_sec_left_front = 0;
double rev_per_sec_left_rear = 0;
double rev_per_sec_right_front = 0;
double rev_per_sec_right_rear = 0;


// MOTOR LOOP CONSTANTS
double interval = 5.0; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;
unsigned long priorTimeLF,priorTimeLR,priorTimeRF,priorTimeRR; // We need to keep track of time for each PID controller separately
double lastSpeedErrorLF,lastSpeedErrorLR,lastSpeedErrorRF,lastSpeedErrorRR; //same with error
double cumErrorLF, cumErrorLR, cumErrorRF, cumErrorRR; // and cumulative error
double maxErr = 20; // chosen arbitrarily for now, students can tune. 
double desVelLF = 10; // will be in inches per sec
double desVelLR = 10; // will be in inches per sec
double desVelRF = 10; // will be in inches per sec
double desVelRR = 10; // will be in inches per sec

// PID CONSTANTS
// LEFT MOTOR - you need to find values. FYI I found good responses with Kp ~ 10x bigger than Ki, and ~3x bigger than Kd. My biggest value was <2.
double kpLF = 1;
double kiLF = 0;
double kdLF = 0;

double kpLR = 1;
double kiLR = 0;
double kdLR = 0;

double kpRF = 1;
double kiRF = 0;
double kdRF = 0;

double kpRR = 1;
double kiRR = 0;
double kdRR = 0;



//=====================================================

void setup() {
  Serial.begin(115200); 
 

}

void loop() {

   unsigned long currentMillis = millis();

      posLeftFrontCount = encoderLeftFront.read();
      posLeftRearCount = encoderLeftRear.read();
        posRightFrontCount = encoderRightFront.read();
        posRightRearCount = encoderRightRear.read();




//      Serial.print("RIGHT: ");
//      Serial.print(posRightCount);
//      Serial.print(',');
//      Serial.print("  ===  LEFT: ");
//      Serial.print(posLeftCount);
//      Serial.print(',');
//      Serial.println();
//      Serial.print(currentMillis);

   if (currentMillis - previousMillis >= interval){
      previousMillis = currentMillis;



     rev_per_sec_left_front = 1000 * (posLeftFrontCount - posLeftFrontCountLast) / encoderResolution / interval;
        rev_per_sec_left_rear = 1000 * (posLeftRearCount - posLeftRearCountLast) / encoderResolution / interval;
        rev_per_sec_right_front = 1000 * (posRightFrontCount - posRightFrontCountLast) / encoderResolution / interval;
        rev_per_sec_right_rear = 1000 * (posRightRearCount - posRightRearCountLast) / encoderResolution / interval;
//     Serial.print("REV/SEC RIGHT: ");
//     Serial.println(rev_per_sec_right);
//     Serial.print("REV/SEC LEFT: ");
//     Serial.println(rev_per_sec_left);
     posLeftFrontRad = rev_per_sec_left_front * 2 * PI; // Same - Rad/sec
        posLeftRearRad = rev_per_sec_left_rear * 2 * PI; // Same - Rad/sec
        posRightFrontRad = rev_per_sec_right_front * 2 * PI; // Same - Rad/sec
        posRightRearRad = rev_per_sec_right_rear * 2 * PI; // Same - Rad/sec

//     Serial.print("RAD/SEC RIGHT: ");
//     Serial.println(posRightRad);
//     Serial.print("RAD/SEC LEFT: ");
//     Serial.println(posLeftRad);
     velLeftFront = rev_per_sec_left_front * d * PI; // Now convert to get inches/sec (tangential velocity)
        velLeftRear = rev_per_sec_left_rear * d * PI; // Same - Inches/sec
        velRightFront = rev_per_sec_right_front * d * PI; // Now convert to get inches/sec (tangential velocity)
        velRightRear = rev_per_sec_right_rear * d * PI; // Same - Inches/sec
//     Serial.print("VEL RIGHT: ");
//     Serial.println(velRight);
//     Serial.print("VEL LEFT: ");
//     Serial.println(velLeft);

     // HERE WILL DO PID AND CREATE NEW MOTOR COMMAND MAPPED TO -400-400 based on max. 
     // COMMENT THIS SECTION OUT TO FIND YOUR MOTORS MAX SPEEDS, 
     newVelLeftFront = drivePIDL(velLeftFront);
        newVelLeftRear = drivePIDL(velLeftRear);
        newVelRightFront = drivePIDR(velRightFront);
        newVelRightRear = drivePIDR(velRightRear);

      // Just some print statements to prove it works. You can comment this out.
      Serial.print("Current Left Front Velocity: ");
      Serial.print(velLeftFront);
      Serial.print('New Left Front Velocity: ');
      Serial.print(newVelLeftFront);
      Serial.print(" Current Left Rear Velocity: ");
        Serial.print(velLeftRear);
        Serial.print('New Left Rear Velocity: ');
        Serial.print(newVelLeftRear);
        Serial.print(" Current Right Front Velocity: ");
        Serial.print(velRightFront);
        Serial.print('New Right Front Velocity: ');
        Serial.print(newVelRightFront);
        Serial.print(" Current Right Rear Velocity: ");
        Serial.print(velRightRear);
        Serial.print('New Right Rear Velocity: ');
        Serial.println(newVelRightRear);


      leftFrontMotor = motorVelToSpeedCommand(newVelLeftFront,leftMotorMax);
        leftRearMotor = motorVelToSpeedCommand(newVelLeftRear,leftMotorMax);
        rightFrontMotor = motorVelToSpeedCommand(newVelRightFront,rightMotorMax);
        rightRearMotor = motorVelToSpeedCommand(newVelRightRear,rightMotorMax);
      /// COMMENT OUT TO HERE FOR FINDING MAX MOTOR SPEED AT 400, You need to add the print statements to get the max speed. 
      
     
     posLeftFrontCountLast = posLeftFrontCount;
        posLeftRearCountLast = posLeftRearCount;
        posRightFrontCountLast = posRightFrontCount;
        posRightRearCountLast = posRightRearCount;
    

     CommandMotors();
   }
}

void CommandMotors(){  

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
  //uncomment to drive motors
}

double drivePIDLF(double curr) {
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;

    currentTime = millis();
    elapsedTime = (double)(currentTime - priorTimeLF);

    error = desVelLF - curr;
    cumErrorLF += error * elapsedTime;

    // Integral Windup
    if(cumErrorLF > maxErr)
        cumErrorLF = maxErr;
    else if(cumErrorLF < -1 * maxErr)
        cumErrorLF = -1 * maxErr;

    rateError = (error - lastSpeedErrorLF) / elapsedTime;

    double out = kpLF * error + kiLF * cumErrorLF + kdLF * rateError;

    lastSpeedErrorLF = error;
    priorTimeLF = currentTime;
    return out;
}

double drivePIDLR(double curr) {
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;

    currentTime = millis();
    elapsedTime = (double)(currentTime - priorTimeLR);

    error = desVelLR - curr;
    cumErrorLR += error * elapsedTime;

    // Integral Windup
    if(cumErrorLR > maxErr)
        cumErrorLR = maxErr;
    else if(cumErrorLR < -1 * maxErr)
        cumErrorLR = -1 * maxErr;

    rateError = (error - lastSpeedErrorLR) / elapsedTime;

    double out = kpLR * error + kiLR * cumErrorLR + kdLR * rateError;

    lastSpeedErrorLR = error;
    priorTimeLR = currentTime;
    return out;
}

double drivePIDRF(double curr) {
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;

    currentTime = millis();
    elapsedTime = (double)(currentTime - priorTimeRF);

    error = desVelRF - curr;
    cumErrorRF += error * elapsedTime;

    // Integral Windup
    if(cumErrorRF > maxErr)
        cumErrorRF = maxErr;
    else if(cumErrorRF < -1 * maxErr)
        cumErrorRF = -1 * maxErr;

    rateError = (error - lastSpeedErrorRF) / elapsedTime;

    double out = kpRF * error + kiRF * cumErrorRF + kdRF * rateError;

    lastSpeedErrorRF = error;
    priorTimeRF = currentTime;
    return out;
}

double drivePIDRR(double curr) {
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;

    currentTime = millis();
    elapsedTime = (double)(currentTime - priorTimeRR);

    error = desVelRR - curr;
    cumErrorRR += error * elapsedTime;

    // Integral Windup
    if(cumErrorRR > maxErr)
        cumErrorRR = maxErr;
    else if(cumErrorRR < -1 * maxErr)
        cumErrorRR = -1 * maxErr;

    rateError = (error - lastSpeedErrorRR) / elapsedTime;

    double out = kpRR * error + kiRR * cumErrorRR + kdRR * rateError;

    lastSpeedErrorRR = error;
    priorTimeRR = currentTime;
    return out;
}


int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    Vel = constrain(Vel,-1*maxVel, maxVel);
    newSpeed = map(Vel,-1*maxVel, maxVel, -400, 400);
    return newSpeed;
}
