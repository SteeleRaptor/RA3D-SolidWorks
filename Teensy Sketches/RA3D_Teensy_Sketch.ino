// =============================================================================================
// FIRMWARE VERSION 6.4
// =============================================================================================
// AR4 Robot Control Software - Teensy 4.1 Motor & Kinematics Controller
// =============================================================================================

/*  
    AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com
*/

// =============================================================================================
// VERSION LOG
// =============================================================================================
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 2.0 - 10/1/22 - added lookahead and spline functionality
// 2.2 - 11/6/22 - added Move V for open cv integrated vision
// 3.0 - 2/3/23 - open loop bypass moved to teensy board / add external axis 8 & 9 / bug fix live jog drift
// 3.1 - 5/10/23 - gcode initial
// 3.2 - 5/12/23 - remove RoboDK kinematics
// 3.3 - 6/4/23 - update geometric kinematics
// 4.0 - 11/5/23 - .txt .ar4 extension, gcode tab, kinematics tab. Initial MK2 release.
// 4.1 - 11/23/23 - bug fix added - R06_neg_matrix[2][3] = -DHparams[5][2]; added to UPdate CMD & GCC diagnostic
// 4.2 - 1/12/24 - bug fix - step direction delay
// 4.3 - 1/21/24 - Gcode to SD card.  Estop button interrupt.
// 4.3.1 - 2/1/24 bug fix - vision snap and find drop down
// 4.4 - 3/2/24 added kinematic error handling
// 4.5 - 6/29/24 simplified drive motors functions with arrays
// 5.0 - 7/14/24 updated kinematics
// 5.1 - 2/15/25 Modbus option
// 5.2 - 6/7/25 Modbus option
// 6.0 - 6/7/25 Virtual Robot
// 6.1 - 8/29/25 updated accel and decel, auto calibrate & microsteps
// 6.2 - 8/29/25 changed bootstrap theme, xbox upgrade
// 6.3 - 10/8/25 - JK - added beta Linux support
// 6.4 - 10/29/25 - added set robot command to store HW and version to eprom, MK4 update, fixed tool jog, re-added 2 step calibration, add servo amp test

// Current firmware version
const char *FIRMWARE_VERSION = "6.4";


// =============================================================================================
// SYSTEM INCLUDES & LIBRARIES
// =============================================================================================
// Mathematical operations and advanced features
#include <math.h>
#include <limits>
#include <avr/pgmspace.h>

// Hardware I/O and communication
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>

// Error handling and Modbus protocol
#include <stdexcept>
#include <ModbusMaster.h>
#include <EEPROM.h>

// =============================================================================================
// COMPILER PRAGMAS - Suppress non-critical warnings
// =============================================================================================
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"
#pragma GCC diagnostic ignored "-Wall"

// =============================================================================================
// DEBUG CONFIGURATION
// =============================================================================================
// Enable/disable debug output
bool DEBUG = false;

// Conditional debug print macros - only print if DEBUG is true
#define DEBUG_PRINT(x) \
  do { \
    if (DEBUG) Serial.print(x); \
  } while (0)
#define DEBUG_PRINTLN(x) \
  do { \
    if (DEBUG) Serial.println(x); \
  } while (0)

// =============================================================================================
// TYPE DEFINITIONS
// =============================================================================================
#define Table_Size 6
typedef float Matrix4x4[16];  // 4x4 transformation matrix for kinematics
typedef float tRobot[66];     // Robot data structure

// =============================================================================================
// SERIAL COMMUNICATION BUFFERS
// =============================================================================================
// Command buffers for lookahead/spline functionality
String cmdBuffer1;
String cmdBuffer2;
String cmdBuffer3;
String inData;     // Current input data
String recData;    // Received data accumulator
String checkData;  // Data validation
String function;   // Command function identifier
volatile byte state = LOW;  // ISR state flag


// =============================================================================================
// TEENSY PIN DEFINITIONS - Motor Control (Step/Direction Pins)
// =============================================================================================
// J1-J6: Main robot joint motors, J7-J9: External/optional axis motors
const int J1stepPin = 0;   // Joint 1 step pulse pin
const int J1dirPin = 1;    // Joint 1 direction control pin
const int J2stepPin = 2;   // Joint 2 step pulse pin
const int J2dirPin = 3;    // Joint 2 direction control pin
const int J3stepPin = 4;   // Joint 3 step pulse pin
const int J3dirPin = 5;    // Joint 3 direction control pin
const int J4stepPin = 6;   // Joint 4 step pulse pin
const int J4dirPin = 7;    // Joint 4 direction control pin
const int J5stepPin = 8;   // Joint 5 step pulse pin
const int J5dirPin = 9;    // Joint 5 direction control pin
const int J6stepPin = 10;  // Joint 6 step pulse pin
const int J6dirPin = 11;   // Joint 6 direction control pin
const int J7stepPin = 12;  // Joint 7 (external axis) step pin
const int J7dirPin = 13;   // Joint 7 (external axis) direction pin
const int J8stepPin = 32;  // Joint 8 (external axis) step pin
const int J8dirPin = 33;   // Joint 8 (external axis) direction pin
const int J9stepPin = 40;  // Joint 9 (external axis) step pin
const int J9dirPin = 41;   // Joint 9 (external axis) direction pin

// =============================================================================================
// TEENSY PIN DEFINITIONS - Calibration/Limit Switches
// =============================================================================================
// Home limit switches for each joint - used during calibration
const int J1calPin = 26;  // Joint 1 home/limit switch
const int J2calPin = 27;  // Joint 2 home/limit switch
const int J3calPin = 28;  // Joint 3 home/limit switch
const int J4calPin = 29;  // Joint 4 home/limit switch
const int J5calPin = 30;  // Joint 5 home/limit switch
const int J6calPin = 31;  // Joint 6 home/limit switch
const int J7calPin = 36;  // Joint 7 home/limit switch
const int J8calPin = 37;  // Joint 8 home/limit switch
const int J9calPin = 38;  // Joint 9 home/limit switch

// Emergency stop button pin
const int EstopPin = 39;  // Emergency stop input (active low)



// =============================================================================================
// ENCODER CONFIGURATION
// =============================================================================================
// Encoder multiplier factors for position feedback scaling
float J1EncoderMultiplier = 10;  // Joint 1 encoder multiplier
float J2EncoderMultiplier = 10;  // Joint 2 encoder multiplier
float J3EncoderMultiplier = 10;  // Joint 3 encoder multiplier
float J4EncoderMultiplier = 10;  // Joint 4 encoder multiplier
float J5EncoderMultiplier = 5;   // Joint 5 encoder multiplier
float J6EncoderMultiplier = 10;  // Joint 6 encoder multiplier
int encOffset = 50;    // Encoder collision detection threshold (steps)

// Encoder instances - (pinA, pinB) for each joint
Encoder J1EncoderPosition(14, 15);  // Joint 1 encoder (pins 14, 15)
Encoder J2EncoderPosition(17, 16);  // Joint 2 encoder (pins 17, 16)
Encoder J3EncoderPosition(19, 18);  // Joint 3 encoder (pins 19, 18)
Encoder J4EncoderPosition(20, 21);  // Joint 4 encoder (pins 20, 21)
Encoder J5EncoderPosition(23, 22);  // Joint 5 encoder (pins 23, 22)
Encoder J6EncoderPosition(24, 25);  // Joint 6 encoder (pins 24, 25)

// Modbus communication master node for industrial device control
ModbusMaster node;



// =============================================================================================
// GLOBAL VARIABLES - AXIS LIMITS (Degrees)
// =============================================================================================
// Define maximum positive and negative angular limits for each joint
float J1axisLimPos = 160;  // Joint 1 positive limit (degrees)
float J1axisLimNeg = 160;  // Joint 1 negative limit (degrees)
float J2axisLimPos = 90;   // Joint 2 positive limit (degrees)
float J2axisLimNeg = 42;   // Joint 2 negative limit (degrees)
float J3axisLimPos = 52;   // Joint 3 positive limit (degrees)
float J3axisLimNeg = 89;   // Joint 3 negative limit (degrees)
float J4axisLimPos = 180;  // Joint 4 positive limit (degrees)
float J4axisLimNeg = 180;  // Joint 4 negative limit (degrees)
float J5axisLimPos = 105;  // Joint 5 positive limit (degrees)
float J5axisLimNeg = 105;  // Joint 5 negative limit (degrees)
float J6axisLimPos = 180;  // Joint 6 positive limit (degrees)
float J6axisLimNeg = 180;  // Joint 6 negative limit (degrees)
float J7axisLimPos = 3450; // Joint 7 (linear) positive limit (mm)
float J7axisLimNeg = 0;    // Joint 7 (linear) negative limit (mm)
float J8axisLimPos = 3450; // Joint 8 (linear) positive limit (mm)
float J8axisLimNeg = 0;    // Joint 8 (linear) negative limit (mm)
float J9axisLimPos = 3450; // Joint 9 (linear) positive limit (mm)
float J9axisLimNeg = 0;    // Joint 9 (linear) negative limit (mm)

// =============================================================================================
// GLOBAL VARIABLES - MOTOR DIRECTIONS
// =============================================================================================
// Motor direction flags: 0=normal, 1=reversed
int J1MotDir = 0;  // Joint 1 motor direction
int J2MotDir = 1;  // Joint 2 motor direction
int J3MotDir = 1;  // Joint 3 motor direction
int J4MotDir = 1;  // Joint 4 motor direction
int J5MotDir = 1;  // Joint 5 motor direction
int J6MotDir = 1;  // Joint 6 motor direction
int J7MotDir = 1;  // Joint 7 motor direction
int J8MotDir = 1;  // Joint 8 motor direction
int J9MotDir = 1;  // Joint 9 motor direction

// Calibration (homing) direction flags: 0=negative, 1=positive limit first
int J1CalDir = 1;  // Joint 1 calibration direction
int J2CalDir = 0;  // Joint 2 calibration direction
int J3CalDir = 1;  // Joint 3 calibration direction
int J4CalDir = 0;  // Joint 4 calibration direction
int J5CalDir = 0;  // Joint 5 calibration direction
int J6CalDir = 1;  // Joint 6 calibration direction
int J7CalDir = 0;  // Joint 7 calibration direction
int J8CalDir = 0;  // Joint 8 calibration direction
int J9CalDir = 0;  // Joint 9 calibration direction


// =============================================================================================
// GLOBAL VARIABLES - AXIS TRAVEL & STEPPER MOTOR CONFIGURATION
// =============================================================================================
// Calculate total axis travel range (positive + negative limits)
float J1AxisDegreeRange = J1axisLimPos + J1axisLimNeg;  // Total J1 travel
float J2axisLim = J2axisLimPos + J2axisLimNeg;  // Total J2 travel
float J3axisLim = J3axisLimPos + J3axisLimNeg;  // Total J3 travel
float J4axisLim = J4axisLimPos + J4axisLimNeg;  // Total J4 travel
float J5axisLim = J5axisLimPos + J5axisLimNeg;  // Total J5 travel
float J6axisLim = J6axisLimPos + J6axisLimNeg;  // Total J6 travel
float J7axisLim = J7axisLimPos + J7axisLimNeg;  // Total J7 travel
float J8axisLim = J8axisLimPos + J8axisLimNeg;  // Total J8 travel
float J9axisLim = J9axisLimPos + J9axisLimNeg;  // Total J9 travel

// =============================================================================================
// MOTOR STEPS PER DEGREE (or mm)
// =============================================================================================
// Stepper motor resolution: steps required to move 1 degree (or 1mm for linear axes)
float J1StepsPerDegree = 88.888;    // Steps per degree for J1
float J2StepDeg = 111.111;   // Steps per degree for J2
float J3StepDeg = 111.111;   // Steps per degree for J3
float J4StepDeg = 99.555;    // Steps per degree for J4
float J5StepDeg = 23.310;    // Steps per degree for J5
float J6StepDeg = 44.444;    // Steps per degree for J6
float J7StepDeg = 14.2857;   // Steps per mm for J7 (linear axis)
float J8StepDeg = 14.2857;   // Steps per mm for J8 (linear axis)
float J9StepDeg = 14.2857;   // Steps per mm for J9 (linear axis)

// =============================================================================================
// TOTAL STEP LIMITS FOR EACH AXIS
// =============================================================================================
// Maximum step count for full range of motion
int J1StepRange = J1AxisDegreeRange * J1StepsPerDegree;  // Total steps for J1 full range
int J2StepRange = J2axisLim * J2StepDeg;  // Total steps for J2 full range
int J3StepRange = J3axisLim * J3StepDeg;  // Total steps for J3 full range
int J4StepRange = J4axisLim * J4StepDeg;  // Total steps for J4 full range
int J5StepRange = J5axisLim * J5StepDeg;  // Total steps for J5 full range
int J6StepRange = J6axisLim * J6StepDeg;  // Total steps for J6 full range
int J7StepRange = J7axisLim * J7StepDeg;  // Total steps for J7 full range
int J8StepRange = J8axisLim * J8StepDeg;  // Total steps for J8 full range
int J9StepRange = J9axisLim * J9StepDeg;  // Total steps for J9 full range

// =============================================================================================
// STEP COUNT AT ZERO POSITION
// =============================================================================================
// Step count when joint is at zero (center) position
int J1zeroStep = J1axisLimNeg * J1StepsPerDegree;  // Steps to reach zero for J1
int J2zeroStep = J2axisLimNeg * J2StepDeg;  // Steps to reach zero for J2
int J3zeroStep = J3axisLimNeg * J3StepDeg;  // Steps to reach zero for J3
int J4zeroStep = J4axisLimNeg * J4StepDeg;  // Steps to reach zero for J4
int J5zeroStep = J5axisLimNeg * J5StepDeg;  // Steps to reach zero for J5
int J6zeroStep = J6axisLimNeg * J6StepDeg;  // Steps to reach zero for J6
int J7zeroStep = J7axisLimNeg * J7StepDeg;  // Steps to reach zero for J7
int J8zeroStep = J8axisLimNeg * J8StepDeg;  // Steps to reach zero for J8
int J9zeroStep = J9axisLimNeg * J9StepDeg;  // Steps to reach zero for J9

// =============================================================================================
// MASTER STEP COUNTERS
// =============================================================================================
// Current step position counters for each joint (initialized at zero position)
// Also, last known position (with or without encoder feedback)
int J1MasterStep = J1zeroStep;  // Master step counter for J1
int J2MasterStep = J2zeroStep;  // Master step counter for J2
int J3MasterStep = J3zeroStep;  // Master step counter for J3
int J4MasterStep = J4zeroStep;  // Master step counter for J4
int J5MasterStep = J5zeroStep;  // Master step counter for J5
int J6MasterStep = J6zeroStep;  // Master step counter for J6
int J7MasterStep = J7zeroStep;  // Master step counter for J7
int J8MasterStep = J8zeroStep;  // Master step counter for J8
int J9MasterStep = J9zeroStep;  // Master step counter for J9



// =============================================================================================
// CALIBRATION OFFSETS
// =============================================================================================
// Home position offset from limit switch (degrees) - fine-tuning after limit detection
float J1calBaseOff = -6.2;  // J1 calibration offset from limit switch
float J2calBaseOff = 3.8;   // J2 calibration offset from limit switch
float J3calBaseOff = 1.4;   // J3 calibration offset from limit switch
float J4calBaseOff = -.8;   // J4 calibration offset from limit switch
float J5calBaseOff = 3.1;   // J5 calibration offset from limit switch
float J6calBaseOff = .5;    // J6 calibration offset from limit switch
float J7calBaseOff = 0;     // J7 calibration offset
float J8calBaseOff = 0;     // J8 calibration offset
float J9calBaseOff = 0;     // J9 calibration offset

// =============================================================================================
// COLLISION DETECTION INDICATORS
// =============================================================================================
// Flags indicating collision/error on each joint (encoder mismatch detection)
int J1collisionTrue = 0;    // J1 collision flag
int J2collisionTrue = 0;    // J2 collision flag
int J3collisionTrue = 0;    // J3 collision flag
int J4collisionTrue = 0;    // J4 collision flag
int J5collisionTrue = 0;    // J5 collision flag
int J6collisionTrue = 0;    // J6 collision flag
int TotalCollision = 0;     // Sum of all collision flags
int KinematicError = 0;     // Inverse kinematics solution not found

// =============================================================================================
// EXTERNAL AXIS (LINEAR/ROTARY) VARIABLES
// =============================================================================================
// Variables for external axes J7, J8, J9
float J7length;             // J7 total length/travel
float J7rot;                // J7 rotation/span
float J7steps;              // J7 total steps

float J8length;             // J8 total length/travel
float J8rot;                // J8 rotation/span
float J8steps;              // J8 total steps

float J9length;             // J9 total length/travel
float J9rot;                // J9 rotation/span
float J9steps;              // J9 total steps

// =============================================================================================
// MOTION CONTROL VARIABLES
// =============================================================================================
float lineDist;             // Linear distance for current motion
String WristCon;            // Wrist configuration parameter
int Quadrant;               // Quadrant for kinematic solutions


// =============================================================================================
// DEBOUNCING & TIMING VARIABLES
// =============================================================================================
// Debounce timers for limit switches - prevent false triggering from electrical noise
unsigned long J1DebounceTime = 0;   // J1 limit switch debounce timer
unsigned long J2DebounceTime = 0;   // J2 limit switch debounce timer
unsigned long J3DebounceTime = 0;   // J3 limit switch debounce timer
unsigned long J4DebounceTime = 0;   // J4 limit switch debounce timer
unsigned long J5DebounceTime = 0;   // J5 limit switch debounce timer
unsigned long J6DebounceTime = 0;   // J6 limit switch debounce timer
unsigned long debounceDelay = 50;   // Debounce delay threshold (milliseconds)

// =============================================================================================
// SPEED & MOTION CONTROL
// =============================================================================================
String Alarm = "0";                 // Error/alarm status flag
String speedViolation = "0";        // Speed limit violation indicator
float minSpeedDelay = 200;          // Minimum speed delay (microseconds between steps)
float maxMMperSec = 192;            // Maximum linear speed (mm/s)
float linWayDistSP = 1;             // Linear waypoint spacing
String debug = "";                  // Debug message string
String flag = "";                   // Status flag string
const int TRACKrotdir = 0;          // Track rotation direction constant
float JogStepInc = 1;               // Jog step increment

// =============================================================================================
// ENCODER STEP COUNTERS
// =============================================================================================
// Raw encoder step values for collision detection
int J1EncSteps;             // J1 encoder current steps
int J2EncSteps;             // J2 encoder current steps
int J3EncSteps;             // J3 encoder current steps
int J4EncSteps;             // J4 encoder current steps
int J5EncSteps;             // J5 encoder current steps
int J6EncSteps;             // J6 encoder current steps

// =============================================================================================
// LOOP MODES (Closed/Open Loop Control)
// =============================================================================================
// 0 = closed loop (use encoder), 1 = open loop (trust step count)
int J1LoopMode;             // J1 loop mode
int J2LoopMode;             // J2 loop mode
int J3LoopMode;             // J3 loop mode
int J4LoopMode;             // J4 loop mode
int J5LoopMode;             // J5 loop mode
int J6LoopMode;             // J6 loop mode
int closedLoopTrue;        // Overall closed loop flag

// =============================================================================================
// ROBOT KINEMATICS - DEGREES OF FREEDOM
// =============================================================================================
#define ROBOT_nDOFs 6  // Number of degrees of freedom (6 joints for main arm)
const int numJoints = 9;  // Total number of joints including external axes

// Kinematics type definitions
typedef float tRobotJoints[ROBOT_nDOFs];  // Joint angles array
typedef float tRobotPose[ROBOT_nDOFs];    // End-effector pose array (x,y,z,rx,ry,rz)

// =============================================================================================
// INVERSE & FORWARD KINEMATICS VARIABLES
// =============================================================================================
// Output: cartesian position from forward kinematics (x,y,z,rx,ry,rz in degrees)
float xyzuvw_Out[ROBOT_nDOFs];  // Output cartesian position (rx,ry,rz in degrees)
// Input: target cartesian position for inverse kinematics
float xyzuvw_In[ROBOT_nDOFs];   // Input target cartesian position
// Temporary cartesian position storage
float xyzuvw_Temp[ROBOT_nDOFs]; // Temporary storage for cartesian coordinates

// =============================================================================================
// JOINT ANGLE VARIABLES
// =============================================================================================
// Output joint angles from inverse kinematics (in degrees)
float JointAnglesInverseKinematic[ROBOT_nDOFs];   // Inverse kinematics solution (degrees)
// Input joint angles for forward kinematics (in degrees)
float CurrentJointAngle[ROBOT_nDOFs];    // Current joint angles (degrees)
// Estimated joint angles for IK solver guidance
float joints_estimate[ROBOT_nDOFs];  // Joint estimate for IK convergence
// Multiple IK solutions for wrist configuration selection
float SolutionMatrix[ROBOT_nDOFs][4];  // IK solutions matrix (up to 4 solutions)

// =============================================================================================
// EXTERNAL AXIS POSITION TRACKING
// =============================================================================================
// Current position of external axes
float J7_pos;  // Joint 7 current position
float J8_pos;  // Joint 8 current position
float J9_pos;  // Joint 9 current position

// Input position targets for external axes
float J7_In;   // Joint 7 target position
float J8_In;   // Joint 8 target position
float J9_In;   // Joint 9 target position

// =============================================================================================
// TRANSFORMATION MATRIX & SEQUENCES
// =============================================================================================
float pose[16];  // 4x4 transformation matrix (homogeneous coordinates)
String moveSequence;  // Move sequence identifier for spline/lookahead


// =============================================================================================
// MOTION PROFILE VARIABLES
// =============================================================================================
// Rounding/corner rounding for smooth trajectory transitions
float rndArcStart[6];   // Start point of rounded arc
float rndArcMid[6];     // Middle point of rounded arc
float rndArcEnd[6];     // End point of rounded arc
float rndCalcCen[6];    // Calculated center of rounding
String rndData;         // Rounding data string
bool rndTrue;           // Flag indicating rounding is active
float rndSpeed;         // Speed during rounding motion

// Spline interpolation
bool splineTrue;        // Flag indicating spline motion active
bool splineEndReceived; // Flag indicating end of spline sequence
bool estopActive;       // Emergency stop active flag

// =============================================================================================
// TOOL FRAME OFFSET (TCP - Tool Center Point)
// =============================================================================================
// Offset from wrist flange to tool center point (TCP) - used in kinematics
float Xtool = 0;   // Tool X offset (mm)
float Ytool = 0;   // Tool Y offset (mm)
float Ztool = 0;   // Tool Z offset (mm)
float RZtool = 0;  // Tool Z rotation offset (degrees)
float RYtool = 0;  // Tool Y rotation offset (degrees)
float RXtool = 0;  // Tool X rotation offset (degrees)

// =============================================================================================
// DENAVIT-HARTENBERG PARAMETERS
// =============================================================================================
// DH parameters: [theta, alpha, d, a] for each joint
// Used for forward kinematics calculations
float DHparams[6][4] = {
  { 0, 0, 169.77, 0 },      // J1: theta, alpha, d, a
  { -90, -90, 0, 64.2 },    // J2: theta, alpha, d, a
  { 0, 0, 0, 305 },         // J3: theta, alpha, d, a
  { 0, -90, 222.63, 0 },    // J4: theta, alpha, d, a
  { 0, 90, 0, 0 },          // J5: theta, alpha, d, a
  { 180, -90, 41, 0 }       // J6: theta, alpha, d, a
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MATRIX OPERATIONS - MACROS FOR EFFICIENT KINEMATICS CALCULATIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// These macros perform 4x4 matrix operations without dynamic allocation
// Used extensively in forward and inverse kinematics calculations

// Matrix multiply: out = inA * inB (4x4 matrix multiplication)
// Computes rotation and translation transformations for joint link frames
#define Matrix_Multiply(out, inA, inB) \
  (out)[0] = (inA)[0] * (inB)[0] + (inA)[4] * (inB)[1] + (inA)[8] * (inB)[2]; \
  (out)[1] = (inA)[1] * (inB)[0] + (inA)[5] * (inB)[1] + (inA)[9] * (inB)[2]; \
  (out)[2] = (inA)[2] * (inB)[0] + (inA)[6] * (inB)[1] + (inA)[10] * (inB)[2]; \
  (out)[3] = 0; \
  (out)[4] = (inA)[0] * (inB)[4] + (inA)[4] * (inB)[5] + (inA)[8] * (inB)[6]; \
  (out)[5] = (inA)[1] * (inB)[4] + (inA)[5] * (inB)[5] + (inA)[9] * (inB)[6]; \
  (out)[6] = (inA)[2] * (inB)[4] + (inA)[6] * (inB)[5] + (inA)[10] * (inB)[6]; \
  (out)[7] = 0; \
  (out)[8] = (inA)[0] * (inB)[8] + (inA)[4] * (inB)[9] + (inA)[8] * (inB)[10]; \
  (out)[9] = (inA)[1] * (inB)[8] + (inA)[5] * (inB)[9] + (inA)[9] * (inB)[10]; \
  (out)[10] = (inA)[2] * (inB)[8] + (inA)[6] * (inB)[9] + (inA)[10] * (inB)[10]; \
  (out)[11] = 0; \
  (out)[12] = (inA)[0] * (inB)[12] + (inA)[4] * (inB)[13] + (inA)[8] * (inB)[14] + (inA)[12]; \
  (out)[13] = (inA)[1] * (inB)[12] + (inA)[5] * (inB)[13] + (inA)[9] * (inB)[14] + (inA)[13]; \
  (out)[14] = (inA)[2] * (inB)[12] + (inA)[6] * (inB)[13] + (inA)[10] * (inB)[14] + (inA)[14]; \
  (out)[15] = 1;

// Matrix inverse: out = inv(in) - Computes inverse for rotation-only transforms
// Used for coordinate frame transformations in kinematics
#define Matrix_Inv(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[4]; \
  (out)[2] = (in)[8]; \
  (out)[3] = 0; \
  (out)[4] = (in)[1]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[9]; \
  (out)[7] = 0; \
  (out)[8] = (in)[2]; \
  (out)[9] = (in)[6]; \
  (out)[10] = (in)[10]; \
  (out)[11] = 0; \
  (out)[12] = -((in)[0] * (in)[12] + (in)[1] * (in)[13] + (in)[2] * (in)[14]); \
  (out)[13] = -((in)[4] * (in)[12] + (in)[5] * (in)[13] + (in)[6] * (in)[14]); \
  (out)[14] = -((in)[8] * (in)[12] + (in)[9] * (in)[13] + (in)[10] * (in)[14]); \
  (out)[15] = 1;

// Matrix copy: out = in - Copies 4x4 matrix
// Simple array copy for transformation matrices
#define Matrix_Copy(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[1]; \
  (out)[2] = (in)[2]; \
  (out)[3] = (in)[3]; \
  (out)[4] = (in)[4]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[6]; \
  (out)[7] = (in)[7]; \
  (out)[8] = (in)[8]; \
  (out)[9] = (in)[9]; \
  (out)[10] = (in)[10]; \
  (out)[11] = (in)[11]; \
  (out)[12] = (in)[12]; \
  (out)[13] = (in)[13]; \
  (out)[14] = (in)[14]; \
  (out)[15] = (in)[15];

// Matrix identity: inout = I - Sets matrix to identity
// Initializes transform matrices to no transformation
#define Matrix_Eye(inout) \
  (inout)[0] = 1; \
  (inout)[1] = 0; \
  (inout)[2] = 0; \
  (inout)[3] = 0; \
  (inout)[4] = 0; \
  (inout)[5] = 1; \
  (inout)[6] = 0; \
  (inout)[7] = 0; \
  (inout)[8] = 0; \
  (inout)[9] = 0; \
  (inout)[10] = 1; \
  (inout)[11] = 0; \
  (inout)[12] = 0; \
  (inout)[13] = 0; \
  (inout)[14] = 0; \
  (inout)[15] = 1;

// Cumulative matrix multiply: inout = inout * inB
// Multiplies transformation sequentially without temporary storage
#define Matrix_Multiply_Cumul(inout, inB) \
  { \
    Matrix4x4 out; \
    Matrix_Multiply(out, inout, inB); \
    Matrix_Copy(inout, out); \
  }

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = { 0 };


//These global variable are also pointers, allowing to put the variables inside the Robot_Data
/// DHM table
float *Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float *Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float *Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

/// Robot lower limits
float *Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

/// Robot upper limits
float *Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float *Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float *Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float *Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float *Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float *Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float *Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float *Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float &Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float &Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float &Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float &Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float &Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float &Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float &Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float &Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool containsNullByte(float value) {
  const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&value);
  for (size_t i = 0; i < sizeof(float); ++i) {
    if (bytes[i] == 0x00) {
      return true;
    }
  }
  return false;
}

bool isValidResult(float value) {
  // Check for NaN or Inf, which are typical results of invalid operations
  return !std::isnan(value) && !std::isinf(value);
}

//This function sets the variable inside Robot_Data to the DHparams
void robot_set_AR() {
  robot_data_reset();

  // Alpha parameters
  Robot_Kin_DHM_L1[DHM_Alpha] = DHparams[0][1] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Alpha] = DHparams[1][1] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Alpha] = DHparams[2][1] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Alpha] = DHparams[3][1] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Alpha] = DHparams[4][1] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Alpha] = DHparams[5][1] * M_PI / 180;

  // Theta parameters
  Robot_Kin_DHM_L1[DHM_Theta] = DHparams[0][0] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Theta] = DHparams[1][0] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Theta] = DHparams[2][0] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Theta] = DHparams[3][0] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Theta] = DHparams[4][0] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Theta] = DHparams[5][0] * M_PI / 180;

  // A parameters
  Robot_Kin_DHM_L1[DHM_A] = DHparams[0][3];
  Robot_Kin_DHM_L2[DHM_A] = DHparams[1][3];
  Robot_Kin_DHM_L3[DHM_A] = DHparams[2][3];
  Robot_Kin_DHM_L4[DHM_A] = DHparams[3][3];
  Robot_Kin_DHM_L5[DHM_A] = DHparams[4][3];
  Robot_Kin_DHM_L6[DHM_A] = DHparams[5][3];

  // D parameters
  Robot_Kin_DHM_L1[DHM_D] = DHparams[0][2];
  Robot_Kin_DHM_L2[DHM_D] = DHparams[1][2];
  Robot_Kin_DHM_L3[DHM_D] = DHparams[2][2];
  Robot_Kin_DHM_L4[DHM_D] = DHparams[3][2];
  Robot_Kin_DHM_L5[DHM_D] = DHparams[4][2];
  Robot_Kin_DHM_L6[DHM_D] = DHparams[5][2];


  Robot_JointLimits_Lower[0] = J1axisLimNeg;
  Robot_JointLimits_Upper[0] = J1axisLimPos;
  Robot_JointLimits_Lower[1] = J2axisLimNeg;
  Robot_JointLimits_Upper[1] = J2axisLimPos;
  Robot_JointLimits_Lower[2] = J3axisLimNeg;
  Robot_JointLimits_Upper[2] = J3axisLimPos;
  Robot_JointLimits_Lower[3] = J4axisLimNeg;
  Robot_JointLimits_Upper[3] = J4axisLimPos;
  Robot_JointLimits_Lower[4] = J5axisLimNeg;
  Robot_JointLimits_Upper[4] = J5axisLimPos;
  Robot_JointLimits_Lower[5] = J6axisLimNeg;
  Robot_JointLimits_Upper[5] = J6axisLimPos;
}

void robot_data_reset() {
  // Reset user base and tool frames
  Matrix_Eye(Robot_BaseFrame);
  Matrix_Eye(Robot_ToolFrame);

  // Reset internal base frame and tool frames
  for (int i = 0; i < 6; i++) {
    Robot_Kin_Base[i] = 0.0;
  }

  // Reset joint senses and joint limits
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Robot_Senses[i] = +1.0;
  }
}

// ============================================================================
// EEPROM Configuration
// ============================================================================

// EEPROM Memory Map
#define EEPROM_MAGIC_ADDR 0            // 4 bytes - magic number to verify valid data
#define EEPROM_DEBUG_ADDR 4            // 1 byte = whether debug is active on boot
#define EEPROM_ROBOT_MODEL_ADDR 5      // 32 bytes - robot model string
#define EEPROM_ROBOT_VERSION_ADDR 37   // 32 bytes - robot version string
#define EEPROM_DRIVER_BOARD_ADDR 69    // 32 bytes - driver board string
#define EEPROM_SERIAL_NUMBER_ADDR 101  // 32 bytes - serial number string
#define EEPROM_ASSET_TAG_ADDR 133      // 32 bytes - asset tag string

#define EEPROM_MAGIC_NUMBER 0x41523401  // "AR4" + version 01


// Default values (used if EEPROM not initialized)
const char *DEFAULT_ROBOT_MODEL = "Unset";
const char *DEFAULT_ROBOT_VERSION = "Unset";
const char *DEFAULT_DRIVER_BOARD = "Unset";
const char *DEFAULT_SERIAL_NUMBER = "Unset";
const char *DEFAULT_ASSET_TAG = "Unset";
const bool DEFAULT_DEBUG = DEBUG;

String robot_model = DEFAULT_ROBOT_MODEL;
String robot_version = DEFAULT_ROBOT_VERSION;
String driver_board = DEFAULT_DRIVER_BOARD;
String serial_number = DEFAULT_SERIAL_NUMBER;
String asset_tag = DEFAULT_ASSET_TAG;


// ============================================================================
// EEPROM Functions
// ============================================================================

bool is_eeprom_initialized() {
  /*
     * Check if EEPROM has been initialized with valid data.
     * 
     * Returns:
     *   true if EEPROM contains valid robot configuration
     *   false if EEPROM is uninitialized or corrupted
     */
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return (magic == EEPROM_MAGIC_NUMBER);
}

void load_debug_from_eeprom() {
  if (is_eeprom_initialized()) {
    // Read from EEPROM
    // If Debug is defined in EEPROM then turn it on now
    bool debugBuf = false;
    EEPROM.get(EEPROM_DEBUG_ADDR, debugBuf);
    if (debugBuf) {
      DEBUG = true;
      DEBUG_PRINTLN("Loaded DEBUG=True from EEPROM - Setting DEBUG to True");
    }
  } else {
    Serial.println("EEPROM not initialized in load_debug");
  }
}

void save_debug_to_eeprom(bool value) {
  bool debugBuf = value;
  DEBUG_PRINT("Saving Value: ");
  DEBUG_PRINT(debugBuf);
  DEBUG_PRINTLN(" to EEPROM");
  EEPROM.put(EEPROM_DEBUG_ADDR, debugBuf);
  bool debugTest;
  EEPROM.get(EEPROM_DEBUG_ADDR, debugTest);

  if (debugTest != value) {
    Serial.println("Error saving Debug Persistence - Values don't match");
  }
}

void load_robot_id_from_eeprom() {
  /*
     * Load robot model and version from EEPROM.
     * If EEPROM not initialized, use default values.
     */
  if (is_eeprom_initialized()) {
    // Read from EEPROM
    char charBuffer[31];
    EEPROM.get(EEPROM_ROBOT_MODEL_ADDR, charBuffer);
    robot_model = charBuffer;
    DEBUG_PRINT("Debug - Loaded Robot Model from EEPROM: ");
    DEBUG_PRINTLN(robot_model);
    EEPROM.get(EEPROM_ROBOT_VERSION_ADDR, charBuffer);
    robot_version = charBuffer;
    DEBUG_PRINT("Debug - Loaded Robot Version from EEPROM: ");
    DEBUG_PRINTLN(robot_version);
    EEPROM.get(EEPROM_DRIVER_BOARD_ADDR, charBuffer);
    driver_board = charBuffer;
    DEBUG_PRINT("Debug - Loaded Driver Board from EEPROM: ");
    DEBUG_PRINTLN(driver_board);
    EEPROM.get(EEPROM_SERIAL_NUMBER_ADDR, charBuffer);
    serial_number = charBuffer;
    DEBUG_PRINT("Debug - Loaded Serial Number from EEPROM: ");
    DEBUG_PRINTLN(serial_number);
    EEPROM.get(EEPROM_ASSET_TAG_ADDR, charBuffer);
    asset_tag = charBuffer;
    DEBUG_PRINT("Debug - Loaded Asset Tag from EEPROM: ");
    DEBUG_PRINTLN(asset_tag);

  } else {
    Serial.println("EEPROM not initialized in load_robot_id");
    robot_model = DEFAULT_ROBOT_MODEL;
    robot_version = DEFAULT_ROBOT_VERSION;
    driver_board = DEFAULT_DRIVER_BOARD;
    serial_number = DEFAULT_SERIAL_NUMBER;
    asset_tag = DEFAULT_ASSET_TAG;
  }
}

void save_robot_id_to_eeprom(const String robot_model, const String robot_version, const String driver_board, const String serial_number, const String asset_tag) {
  /*
     * Save robot model and version to EEPROM.
     * 
     * Args:
     *   robot_model: Robot model string (max 31 chars)
     *   robot_version: Robot version string (max 31 chars)
     *   driver_board: Driver board string (max 31 chars)
     *   serial_number: Serial number string (max 31 chars)
     *   asset_tag: Asset Tag string (max 31 chars)
     */
  // Write magic number
  uint32_t magic = EEPROM_MAGIC_NUMBER;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);

  char charBuffer[32] = { 0 };

  // Write robot model
  if (robot_model != "NA") {
    robot_model.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ROBOT_MODEL_ADDR, charBuffer);
  }

  // Write robot version
  if (robot_version != "NA") {
    robot_version.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ROBOT_VERSION_ADDR, charBuffer);
  }

  // Write driver board
  if (driver_board != "NA") {
    driver_board.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_DRIVER_BOARD_ADDR, charBuffer);
  }

  // Write Serial Number
  if (serial_number != "NA") {
    serial_number.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_SERIAL_NUMBER_ADDR, charBuffer);
  }

  // Write Asset Tag
  if (asset_tag != "NA") {
    asset_tag.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ASSET_TAG_ADDR, charBuffer);
  }
}

void reboot() {
  DEBUG_PRINT("Rebooting Driver Board: ");
  DEBUG_PRINTLN(driver_board);
  if (driver_board.indexOf("Teensy") >= 0) {
    DEBUG_PRINTLN("Teensy 3.x / 4.x: ARM system reset");
    SCB_AIRCR = 0x05FA0004;
    while (true)
      ;
  } else {
    // Unknown type — fallback or safe no-op
    Serial.println("Unknown board type, no reboot performed.");
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Persistent Hardware / Query Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

void handle_hello_command() {
  /*
     * Responds to HELLO command with system information in JSON format.
     * Reads robot model/version from EEPROM (or uses defaults).
     * 
     * Command: HELLO\n
     * Response: {"DriverModel":"Teensy 4.1","DriverVersion":"6.3","RobotModel":"AR4","RobotVersion":"Mk3"}\n
     */

  String response = "{";
  response += "\"DriverModel\":\"" + String(driver_board) + "\",";
  response += "\"FirmwareVersion\":\"" + String(FIRMWARE_VERSION) + "\",";
  response += "\"RobotModel\":\"" + String(robot_model) + "\",";
  response += "\"RobotVersion\":\"" + String(robot_version) + "\",";
  response += "\"SerialNumber\":\"" + String(serial_number) + "\",";
  response += "\"AssetTag\":\"" + String(asset_tag) + "\"";
  response += "}";

  Serial.println(response);
}


void handle_set_robot_id_command(String robot_model, String robot_version, String driver_board, String serial_number, String asset_tag) {
  /*
     * Set robot model and version, save to EEPROM.
     * 
     * Response: Done\n (on success) or Error\n (on failure)
     */

  if (robot_model.length() == 0) {
    DEBUG_PRINTLN("No Robot Model Provided - Not Setting");
    robot_model = "NA";
  } else if (robot_model.length() > 31) {
    Serial.println("Error: Robot Model too long (max 31 chars)");
    return;
  }

  if (robot_version.length() == 0) {
    DEBUG_PRINTLN("No Robot Version Provded - Not Setting");
    robot_version = "NA";
    if (robot_version.length() > 31) {
      Serial.println("Error: Robot Version too long (max 31 chars)");
      return;
    }
  }

  if (driver_board.length() == 0) {
    DEBUG_PRINTLN("No Driver Board Provded - Not Setting");
    driver_board = "NA";
  } else if (driver_board.length() > 31) {
    Serial.println("Error: Driver Board too long (max 31 chars)");
    return;
  }

  if (serial_number.length() == 0) {
    DEBUG_PRINTLN("No Serial Number - Not Setting");
    serial_number = "NA";
  } else if (serial_number.length() > 31) {
    Serial.println("Error: Version too long (max 31 chars)");
    return;
  }

  if (asset_tag.length() == 0) {
    DEBUG_PRINTLN("No Asset Tag Provded - Not Setting");
    asset_tag = "NA";
  } else if (asset_tag.length() > 31) {
    Serial.println("Error: Asset Tag too long (max 31 chars)");
    return;
  }

  robot_model.trim();
  robot_version.trim();
  driver_board.trim();
  serial_number.trim();
  asset_tag.trim();

  DEBUG_PRINT("Debug - Setting Robot Model: ");
  DEBUG_PRINTLN(robot_model);
  DEBUG_PRINT("Debug - Setting Robot Version: ");
  DEBUG_PRINTLN(robot_version);
  DEBUG_PRINT("Debug - Setting Driver Board: ");
  DEBUG_PRINTLN(driver_board);
  DEBUG_PRINT("Debug - Setting Serial Number: ");
  DEBUG_PRINTLN(serial_number);
  DEBUG_PRINT("Debug - Setting Asset Tag: ");
  DEBUG_PRINTLN(asset_tag);

  // Save to EEPROM
  save_robot_id_to_eeprom(robot_model, robot_version, driver_board, serial_number, asset_tag);

  Serial.println("Done");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRICE OPERATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
bool robot_joints_valid(const T joints[ROBOT_nDOFs]) {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if (joints[i] < -Robot_JointLimits_Lower[i] || joints[i] > Robot_JointLimits_Upper[i]) {
      return false;
    }
  }
  return true;
}


//This function returns a 4x4 matrix as an argument (pose) following the modified DH rules for the inputs T rx, T tx, T rz and T tz source : https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
template<typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose) {
  T crx;
  T srx;
  T crz;
  T srz;
  crx = cos(rx);
  srx = sin(rx);
  crz = cos(rz);
  srz = sin(rz);
  pose[0] = crz;
  pose[4] = -srz;
  pose[8] = 0.0;
  pose[12] = tx;
  pose[1] = crx * srz;
  pose[5] = crx * crz;
  pose[9] = -srx;
  pose[13] = -tz * srx;
  pose[2] = srx * srz;
  pose[6] = crz * srx;
  pose[10] = crx;
  pose[14] = tz * crx;
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix and return it as an argument.
template<typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose) {
  T srx;
  T crx;
  T sry;
  T cry;
  T srz;
  T crz;
  T H_tmp;
  srx = sin(xyzwpr[3]);
  crx = cos(xyzwpr[3]);
  sry = sin(xyzwpr[4]);
  cry = cos(xyzwpr[4]);
  srz = sin(xyzwpr[5]);
  crz = cos(xyzwpr[5]);
  pose[0] = cry * crz;
  pose[4] = -cry * srz;
  pose[8] = sry;
  pose[12] = xyzwpr[0];
  H_tmp = crz * srx;
  pose[1] = crx * srz + H_tmp * sry;
  crz *= crx;
  pose[5] = crz - srx * sry * srz;
  pose[9] = -cry * srx;
  pose[13] = xyzwpr[1];
  pose[2] = srx * srz - crz * sry;
  pose[6] = H_tmp + crx * sry * srz;
  pose[10] = crx * cry;
  pose[14] = xyzwpr[2];
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template<typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6]) {
  T sin_angle;
  T angle;
  T vector[3];
  int iidx;
  int vector_tmp;
  signed char b_I[9];
  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];
  sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
  if (sin_angle <= -1.0) {
    sin_angle = -1.0;
  }

  if (sin_angle >= 1.0) {
    sin_angle = 1.0;
  }

  angle = acos(sin_angle);
  if (angle < 1.0E-6) {
    vector[0] = 0.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
  } else {
    sin_angle = sin(angle);
    if (abs(sin_angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      sin_angle = pose[0];
      iidx = 0;
      if (pose[0] < pose[5]) {
        sin_angle = pose[5];
        iidx = 1;
      }

      if (sin_angle < pose[10]) {
        sin_angle = pose[10];
        iidx = 2;
      }

      for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
        b_I[vector_tmp] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      sin_angle = 2.0 * (1.0 + sin_angle);
      if (sin_angle <= 0.0) {
        sin_angle = 0.0;
      } else {
        sin_angle = sqrt(sin_angle);
      }

      vector_tmp = iidx << 2;
      vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) / sin_angle;
      vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                  / sin_angle;
      vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                  / sin_angle;
      angle = M_PI;
    } else {
      sin_angle = 1.0 / (2.0 * sin_angle);
      vector[0] = (pose[6] - pose[9]) * sin_angle;
      vector[1] = (pose[8] - pose[2]) * sin_angle;
      vector[2] = (pose[1] - pose[4]) * sin_angle;
    }
  }

  sin_angle = angle * 180.0 / M_PI;
  out[3] = vector[0] * sin_angle * M_PI / 180.0;
  out[4] = vector[1] * sin_angle * M_PI / 180.0;
  out[5] = vector[2] * sin_angle * M_PI / 180.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix using UR euler rules and return it as an argument.
template<typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose) {
  T s;
  T angle;
  T axisunit[3];
  T ex;
  T c;
  T pose_tmp;
  T b_pose_tmp;
  s = sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] * xyzuvw[5]);
  angle = s * 180.0 / M_PI;
  if (abs(angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
    memset(&pose[0], 0, sizeof(T) << 4);
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;
  } else {
    axisunit[1] = abs(xyzuvw[4]);
    axisunit[2] = abs(xyzuvw[5]);
    ex = abs(xyzuvw[3]);
    if (abs(xyzuvw[3]) < axisunit[1]) {
      ex = axisunit[1];
    }

    if (ex < axisunit[2]) {
      ex = axisunit[2];
    }

    if (ex < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      memset(&pose[0], 0, sizeof(T) << 4);
      pose[0] = 1.0;
      pose[5] = 1.0;
      pose[10] = 1.0;
      pose[15] = 1.0;
    } else {
      axisunit[0] = xyzuvw[3] / s;
      axisunit[1] = xyzuvw[4] / s;
      axisunit[2] = xyzuvw[5] / s;
      s = angle * 3.1415926535897931 / 180.0;
      c = cos(s);
      s = sin(s);
      angle = axisunit[0] * axisunit[0];
      pose[0] = angle + c * (1.0 - angle);
      angle = axisunit[0] * axisunit[1] * (1.0 - c);
      ex = axisunit[2] * s;
      pose[4] = angle - ex;
      pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
      b_pose_tmp = axisunit[1] * s;
      pose[8] = pose_tmp + b_pose_tmp;
      pose[1] = angle + ex;
      angle = axisunit[1] * axisunit[1];
      pose[5] = angle + (1.0 - angle) * c;
      angle = axisunit[1] * axisunit[2] * (1.0 - c);
      ex = axisunit[0] * s;
      pose[9] = angle - ex;
      pose[2] = pose_tmp - b_pose_tmp;
      pose[6] = angle + ex;
      angle = axisunit[2] * axisunit[2];
      pose[10] = angle + (1.0 - angle) * c;
      pose[3] = 0.0;
      pose[7] = 0.0;
      pose[11] = 0.0;
      pose[15] = 1.0;
    }
  }

  pose[12] = xyzuvw[0];
  pose[13] = xyzuvw[1];
  pose[14] = xyzuvw[2];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FOWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function input the JxangleIn into an array, send it to the foward kinematic solver and output the result into the position variables
void SolveFowardKinematics() {

  // Load and configure Denavit-Hartenberg parameters for robot kinematics
  robot_set_AR();

  // Allocate array for output Cartesian position (x,y,z,rx,ry,rz)
  float target_xyzuvw[6];
  // Allocate array for joint angles to be input to FK solver (in degrees)
  float joints[ROBOT_nDOFs];

  // Copy current joint angles from global array to local array
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    // Copy each joint angle for processing
    joints[i] = CurrentJointAngle[i];
  }

  // Call forward kinematics solver with joint angles to get Cartesian position
  forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);

  // Store calculated X position (in mm)
  xyzuvw_Out[0] = target_xyzuvw[0];
  // Store calculated Y position (in mm)
  xyzuvw_Out[1] = target_xyzuvw[1];
  // Store calculated Z position (in mm)
  xyzuvw_Out[2] = target_xyzuvw[2];
  // Store calculated Rx rotation (convert from radians to degrees)
  xyzuvw_Out[3] = target_xyzuvw[3] / M_PI * 180;
  // Store calculated Ry rotation (convert from radians to degrees)
  xyzuvw_Out[4] = target_xyzuvw[4] / M_PI * 180;
  // Store calculated Rz rotation (convert from radians to degrees)
  xyzuvw_Out[5] = target_xyzuvw[5] / M_PI * 180;
}



template<typename T>
void forward_kinematics_arm(const T *joints, Matrix4x4 pose) {
  xyzwpr_2_pose(Robot_Kin_Base, pose);
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Matrix4x4 hi;
    float *dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
    T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
    DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
    Matrix_Multiply_Cumul(pose, hi);
  }
  Matrix4x4 tool_pose;
  xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
  Matrix_Multiply_Cumul(pose, tool_pose);
}


template<typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
  Matrix4x4 pose;
  forward_kinematics_robot(joints, pose);  //send the joints values and return the pose matrix as an argument
  pose_2_xyzuvw(pose, target_xyzuvw);      //send the pose matrix and return the xyzuvw values in an array as an argument
}

//Calculate de foward kinematic of the robot without the tool
template<typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
  Matrix4x4 invBaseFrame;
  Matrix4x4 pose_arm;
  Matrix_Inv(invBaseFrame, Robot_BaseFrame);  // invRobot_Tool could be precalculated, the tool does not change so often
  forward_kinematics_arm(joints, pose_arm);
  Matrix_Multiply(target, invBaseFrame, pose_arm);
  Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//vestigal function
void updatejoints() {

  for (int i = 0; i > ROBOT_nDOFs; i++) {
    CurrentJointAngle[i] = JointAnglesInverseKinematic[i];
  }
}

void JointEstimate() {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = CurrentJointAngle[i];
  }
}

void SolveInverseKinematics() {

  // Allocate array for joint angles solution
  float joints[ROBOT_nDOFs];
  // Allocate array for target Cartesian position (x,y,z,rx,ry,rz)
  float target[6];

  // Buffer to store solution joint angles
  float solbuffer[ROBOT_nDOFs] = { 0 };
  // Counter for number of valid inverse kinematics solutions found
  int NumberOfSol = 0;
  // Index of selected solution from multiple possibilities
  int solVal = 0;

  // Clear any previous kinematic error condition
  KinematicError = 0;

  // Estimate current joint angles as initial guess for IK solver
  JointEstimate();
  // Extract X position from input target Cartesian command
  target[0] = xyzuvw_In[0];
  // Extract Y position from input target
  target[1] = xyzuvw_In[1];
  // Extract Z position from input target
  target[2] = xyzuvw_In[2];
  // Extract Rx rotation (convert from degrees to radians)
  target[3] = xyzuvw_In[3] * M_PI / 180;
  // Extract Ry rotation (convert from degrees to radians)
  target[4] = xyzuvw_In[4] * M_PI / 180;
  // Extract Rz rotation (convert from degrees to radians)
  target[5] = xyzuvw_In[5] * M_PI / 180;

  // Serial.println("X : " + String(target[0]) + " Y : " + String(target[1]) + " Z : " + String(target[2]) + " rx : " + String(xyzuvw_In[3]) + " ry : " + String(xyzuvw_In[4]) + " rz : " + String(xyzuvw_In[5]));

  // Try multiple wrist configurations to find all valid solutions
  // Sweep J5 (wrist rotation) from -90 to +90 degrees in 30-degree increments
  for (int i = -3; i <= 3; i++) {
    // Set wrist joint estimate to current sweep value
    joints_estimate[4] = i * 30;
    // Attempt to solve inverse kinematics with this wrist configuration
    int success = inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate);
    // Check if valid solution was found
    if (success) {
      // Check if new solution is different from previous one
      if (solbuffer[4] != joints[4]) {
        // Validate that solution is within joint limits
        if (robot_joints_valid(joints)) {
          // Copy solution to buffer for storage
          for (int j = 0; j < ROBOT_nDOFs; j++) {
            // Store joint angle from solution
            solbuffer[j] = joints[j];
            // Add to solution matrix for later selection
            SolutionMatrix[j][NumberOfSol] = solbuffer[j];
          }
          // Check if solution count has not exceeded maximum
          if (NumberOfSol <= 6) {
            // Increment number of valid solutions found
            NumberOfSol++;
          }
        }
      }
    } else {
      // IK solver failed for this configuration
      KinematicError = 1;
    }
  }

  // Restore original J5 estimate for solution selection
  joints_estimate[4] = CurrentJointAngle[4];

  // Default to first solution
  solVal = 0;
  // Evaluate which solution is closest to current joint configuration
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    // Check if first solution is significantly different from estimate
    if ((abs(joints_estimate[i] - SolutionMatrix[i][0]) > 20) and NumberOfSol > 1) {
      // Use second solution instead
      solVal = 1;
    } else if ((abs(joints_estimate[i] - SolutionMatrix[i][1]) > 20) and NumberOfSol > 1) {
      // Use first solution instead
      solVal = 0;
    }

    // Serial.println(String(i) + "  Joint estimate : " + String(joints_estimate[i]) + " // Joint sol 1 : " + String(SolutionMatrix[i][0]) + " // Joint sol 2 : " + String(SolutionMatrix[i][1]));
  }

  // Check if no valid solutions were found
  if (NumberOfSol == 0) {
    // Set error flag indicating unreachable target position
    KinematicError = 1;
  }

  // Serial.println("Sol : " + String(solVal) + " Nb sol : " + String(NumberOfSol));

  // Copy selected solution to output array
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    // Copy joint angle from selected solution to output
    JointAnglesInverseKinematic[i] = SolutionMatrix[i][solVal];
  }
}





template<typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T *joints_estimate) {
  Matrix4x4 invToolFrame;
  Matrix4x4 pose_arm;
  int nsol;
  Matrix_Inv(invToolFrame, Robot_ToolFrame);  // invRobot_Tool could be precalculated, the tool does not change so often
  Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
  Matrix_Multiply_Cumul(pose_arm, invToolFrame);
  if (joints_estimate != nullptr) {
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
  } else {
    // Warning! This is dangerous if joints does not have a valid/reasonable result
    T joints_approx[6];
    memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
  }
  if (nsol == 0) {
    return 0;
  }

  return 1;
}


template<typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate) {

  Matrix4x4 pose;
  xyzuvw_2_pose(target_xyzuvw1, pose);
  return inverse_kinematics_robot(pose, joints, joints_estimate);
}


template<typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int *nsol) {
  int i0;
  T base[16];
  T joints_approx[6];
  T tool[16];
  int i;
  T Hout[16];
  T b_Hout[9];
  T dv0[4];
  bool guard1 = false;
  T make_sqrt;
  T P04[4];
  T q1;
  int i1;
  T c_Hout[16];
  T k2;
  T k1;
  T ai;
  T B;
  T C;
  T s31;
  T c31;
  T q13_idx_2;
  T bb_div_cc;
  T q13_idx_0;
  for (i0 = 0; i0 < 6; i0++) {
    joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
  }

  //debug = String(Robot_Data[13]) + " * " + String(Robot_Data[19]) + " * " + String(Robot_Data[21]);

  xyzwpr_2_pose(*(T(*)[6]) & DK[36], base);
  xyzwpr_2_pose(*(T(*)[6]) & DK[42], tool);
  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    Hout[i] = base[i0];
    Hout[1 + i] = base[i0 + 4];
    Hout[2 + i] = base[i0 + 8];
    Hout[3 + i] = base[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    Hout[3 + i] = 0.0;
    b_Hout[3 * i0] = -Hout[i];
    b_Hout[1 + 3 * i0] = -Hout[1 + i];
    b_Hout[2 + 3 * i0] = -Hout[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) + b_Hout[i0 + 6] * base[14];
  }

  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    base[i] = tool[i0];
    base[1 + i] = tool[i0 + 4];
    base[2 + i] = tool[i0 + 8];
    base[3 + i] = tool[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    base[3 + i] = 0.0;
    b_Hout[3 * i0] = -base[i];
    b_Hout[1 + 3 * i0] = -base[1 + i];
    b_Hout[2 + 3 * i0] = -base[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) + b_Hout[i0 + 6] * tool[14];
  }

  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = -DK[33];
  dv0[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) + Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 + i1];
    }

    P04[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) + c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 + i1];
      tool[i0 + i1] = make_sqrt;
      P04[i0] += make_sqrt * dv0[i];
    }
  }

  guard1 = false;
  if (DK[9] == 0.0) {
    q1 = atan2(P04[1], P04[0]);
    guard1 = true;
  } else {
    make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
    if (make_sqrt < 0.0) {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    } else {
      q1 = atan2(P04[1], P04[0]) - atan2(DK[9], sqrt(make_sqrt));
      guard1 = true;
    }
  }

  if (guard1) {
    k2 = P04[2] - DK[3];
    k1 = (cos(q1) * P04[0] + sin(q1) * P04[1]) - DK[7];
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19];
    B = 2.0 * DK[21] * DK[13];
    C = 2.0 * DK[19] * DK[13];
    s31 = 0.0;
    c31 = 0.0;
    if (C == 0.0) {
      s31 = -ai / B;
      make_sqrt = 1.0 - s31 * s31;
      if (make_sqrt >= 0.0) {
        c31 = sqrt(make_sqrt);
      }
    } else {
      q13_idx_2 = C * C;
      bb_div_cc = B * B / q13_idx_2;
      make_sqrt = 2.0 * ai * B / q13_idx_2;
      make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0));
      if (make_sqrt >= 0.0) {
        s31 = (-2.0 * ai * B / q13_idx_2 + sqrt(make_sqrt)) / (2.0 * (1.0 + bb_div_cc));
        c31 = (ai + B * s31) / C;
      }
    }

    if ((make_sqrt >= 0.0) && (abs(s31) <= 1.0)) {
      B = atan2(s31, c31);
      make_sqrt = cos(B);
      ai = sin(B);
      C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
      make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
      q13_idx_0 = q1 + -DK[2];
      k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] - M_PI / 2);
      q13_idx_2 = B + -DK[14];
      bb_div_cc = joints_approx[3] * M_PI / 180.0 - (-DK[20]);
      q1 = q13_idx_0 + DK[2];
      B = k2 + DK[8];
      C = q13_idx_2 + DK[14];
      make_sqrt = B + C;
      s31 = cos(make_sqrt);
      c31 = cos(q1);
      Hout[0] = s31 * c31;
      ai = sin(q1);
      Hout[4] = s31 * ai;
      make_sqrt = sin(make_sqrt);
      Hout[8] = -make_sqrt;
      Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * cos(C);
      Hout[1] = -sin(B + C) * c31;
      Hout[5] = -sin(B + C) * ai;
      Hout[9] = -s31;
      Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * sin(C);
      Hout[2] = -ai;
      Hout[6] = c31;
      Hout[10] = 0.0;
      Hout[14] = 0.0;
      Hout[3] = 0.0;
      Hout[7] = 0.0;
      Hout[11] = 0.0;
      Hout[15] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
          i1 = i << 2;
          base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) + Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3 + i1];
        }
      }

      make_sqrt = 1.0 - base[9] * base[9];
      if (make_sqrt <= 0.0) {
        make_sqrt = 0.0;
      } else {
        make_sqrt = sqrt(make_sqrt);
      }

      if (make_sqrt < 1.0E-6) {
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(bb_div_cc);
        ai = cos(bb_div_cc);
        make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2] - ai * base[0]);
      } else if (joints_approx[4] >= 0.0) {
        bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      } else {
        bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
        C = atan2(-make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      }

      joints[0] = q13_idx_0;
      joints[3] = bb_div_cc + -DK[20];
      joints[1] = k2;
      joints[4] = C + -DK[26];
      joints[2] = q13_idx_2;
      joints[5] = make_sqrt + (-DK[32] + M_PI);
      make_sqrt = joints[5];
      if (joints[5] > 3.1415926535897931) {
        make_sqrt = joints[5] - M_PI * 2;
      } else {
        if (joints[5] <= -M_PI) {
          make_sqrt = joints[5] + M_PI * 2;
        }
      }

      joints[5] = make_sqrt;
      for (i0 = 0; i0 < 6; i0++) {
        joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / M_PI);
      }

      *nsol = 1.0;
    } else {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    }
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// POSITION FEEDBACK & FORWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void sendRobotPos()
//
// Calculates current Cartesian position from joint angles using forward kinematics,
// then reports complete robot state to host via serial. Includes:
// - Joint angles (J1-J6 in degrees)
// - Cartesian position and orientation (X,Y,Z,Rx,Ry,Rz)
// - External axis positions (J7,J8,J9)
// - Status flags (speed violation, debug info, error codes)
//
void sendRobotPos() {

  // Update position from internal step counters
  updatePos();

  // Build position response string with all joint and cartesian data
  // Format: A<J1>B<J2>...L<Rz>M<speedViolation>N<debug>O<flag>P<J7>Q<J8>R<J9>
  String sendPos = "A" + String(CurrentJointAngle[0], 3) + "B" + String(CurrentJointAngle[1], 3) + "C" + String(CurrentJointAngle[2], 3) + "D" + String(CurrentJointAngle[3], 3) + "E" + String(CurrentJointAngle[4], 3) + "F" + String(CurrentJointAngle[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  // Small delay before transmission for serial stability
  delay(5);
  // Send position response to host
  Serial.println(sendPos);
  // Clear speed violation flag after reporting
  speedViolation = "0";
  // Clear debug message after sending
  flag = "";
}

// void sendRobotPosSpline()
//
// Optimized position reporting for spline mode operation. Sends current state
// for lookahead trajectory planning without clearing speed violation flag.
//
void sendRobotPosSpline() {

  updatePos();

  String sendPos = "A" + String(CurrentJointAngle[0], 3) + "B" + String(CurrentJointAngle[1], 3) + "C" + String(CurrentJointAngle[2], 3) + "D" + String(CurrentJointAngle[3], 3) + "E" + String(CurrentJointAngle[4], 3) + "F" + String(CurrentJointAngle[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
}

void updatePos() {
  if (closedLoopTrue){
    // Read encoder position and update internal step counters from encoder feedback
    readEncoders();
  }
  // Convert J1 step count to angle in degrees (relative to zero position)
  CurrentJointAngle[0] = (J1MasterStep - J1zeroStep) / J1StepsPerDegree;
  // Convert J2 step count to angle in degrees
  CurrentJointAngle[1] = (J2MasterStep - J2zeroStep) / J2StepDeg;
  // Convert J3 step count to angle in degrees
  CurrentJointAngle[2] = (J3MasterStep - J3zeroStep) / J3StepDeg;
  // Convert J4 step count to angle in degrees
  CurrentJointAngle[3] = (J4MasterStep - J4zeroStep) / J4StepDeg;
  // Convert J5 step count to angle in degrees
  CurrentJointAngle[4] = (J5MasterStep - J5zeroStep) / J5StepDeg;
  // Convert J6 step count to angle in degrees
  CurrentJointAngle[5] = (J6MasterStep - J6zeroStep) / J6StepDeg;

  // Convert J7 step count to position (linear or rotary)
  J7_pos = (J7MasterStep - J7zeroStep) / J7StepDeg;
  // Convert J8 step count to position
  J8_pos = (J8MasterStep - J8zeroStep) / J8StepDeg;
  // Convert J9 step count to position
  J9_pos = (J9MasterStep - J9zeroStep) / J9StepDeg;

  // Solve forward kinematics to get Cartesian position from joint angles
  SolveFowardKinematics();
}


//Update Master step and send postion through serial
void correctRobotPos() {

  // Read encoder position and update internal step counters from encoder feedback
  // This synchronizes the system with actual motor positions
  // Read J1 encoder and scale by encoder multiplier
  J1MasterStep = J1EncoderPosition.read() / J1EncoderMultiplier;
  // Read J2 encoder and scale by multiplier
  J2MasterStep = J2EncoderPosition.read() / J2EncoderMultiplier;
  // Read J3 encoder and scale by multiplier
  J3MasterStep = J3EncoderPosition.read() / J3EncoderMultiplier;
  // Read J4 encoder and scale by multiplier
  J4MasterStep = J4EncoderPosition.read() / J4EncoderMultiplier;
  // Read J5 encoder and scale by multiplier
  J5MasterStep = J5EncoderPosition.read() / J5EncoderMultiplier;
  // Read J6 encoder and scale by multiplier
  J6MasterStep = J6EncoderPosition.read() / J6EncoderMultiplier;

  // Convert updated step counts to joint angles (degrees)
  // Calculate J1 angle from encoder-based step count
  CurrentJointAngle[0] = (J1MasterStep - J1zeroStep) / J1StepsPerDegree;
  // Calculate J2 angle from encoder-based step count
  CurrentJointAngle[1] = (J2MasterStep - J2zeroStep) / J2StepDeg;
  // Calculate J3 angle from encoder-based step count
  CurrentJointAngle[2] = (J3MasterStep - J3zeroStep) / J3StepDeg;
  // Calculate J4 angle from encoder-based step count
  CurrentJointAngle[3] = (J4MasterStep - J4zeroStep) / J4StepDeg;
  // Calculate J5 angle from encoder-based step count
  CurrentJointAngle[4] = (J5MasterStep - J5zeroStep) / J5StepDeg;
  // Calculate J6 angle from encoder-based step count
  CurrentJointAngle[5] = (J6MasterStep - J6zeroStep) / J6StepDeg;

  // Solve forward kinematics to get Cartesian position from corrected joint angles
  SolveFowardKinematics();

  // Build corrected position response string
  String sendPos = "A" + String(CurrentJointAngle[0], 3) + "B" + String(CurrentJointAngle[1], 3) + "C" + String(CurrentJointAngle[2], 3) + "D" + String(CurrentJointAngle[3], 3) + "E" + String(CurrentJointAngle[4], 3) + "F" + String(CurrentJointAngle[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  // Brief delay for serial stability
  delay(5);
  // Send corrected position to host
  Serial.println(sendPos);
  // Clear speed violation flag
  speedViolation = "0";
  // Clear status flag
  flag = "";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeSD(String filename, String info) {
  SD.begin(BUILTIN_SDCARD);
  const char *fn = filename.c_str();
  File gcFile = SD.open(fn, FILE_WRITE);
  if (gcFile) {
    //Serial.print("Writing to test.txt...");
    gcFile.println(info);
    gcFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("EG");
  }
}

void deleteSD(String filename) {
  SD.begin(BUILTIN_SDCARD);
  const char *fn = filename.c_str();
  SD.remove(fn);
}

void printDirectory(File dir, int numTabs) {
  String filesSD;
  while (true) {

    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      Serial.println(filesSD);
      break;
    }
    if (entry.name() != "System Volume Information") {
      filesSD += entry.name();
      filesSD += ",";
    }
    entry.close();
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(const int steps[], float SpeedVal) {

  // Debounce time in microseconds (3ms to filter noise)
  const unsigned long DEBOUNCE_US = 3000;  // 3 ms
  // Track when each joint's limit switch first went HIGH
  unsigned long firstHighUs[numJoints] = { 0 };
  // Calculate step gap (delay between pulses) for calibration speed
  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  // Define arrays for calibration directions, motor directions, and direction pins
  // Expected limit switch state (HIGH = at limit)
  const uint8_t limitSensor[numJoints] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
  // Calibration direction for each joint (0=neg limit first, 1=pos limit first)
  int calDir[numJoints] = { J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir, J6CalDir, J7CalDir, J8CalDir, J9CalDir };
  // Motor direction configuration
  int motDir[numJoints] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  // Direction control pins for each motor
  int dirPins[numJoints] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };

  // Define arrays for the current state, calibration pins, step pins, steps, completion status, and steps done
  // Current read state of each limit switch pin
  int curState[numJoints];
  // Calibration/limit switch pins for each joint
  int calPins[numJoints] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
  // Step pulse pins for each joint
  int stepPins[numJoints] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  // Counter for steps executed on each joint
  int stepsDone[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  // Flag indicating each joint has completed homing (hit limit)
  int complete[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // Set direction pins for all joints based on calibration direction
  for (int i = 0; i < numJoints; i++) {
    // Check if calibration and motor direction match
    if ((calDir[i] == 1 && motDir[i] == 1) || (calDir[i] == 0 && motDir[i] == 0)) {
      // Set direction pin HIGH for forward calibration
      digitalWrite(dirPins[i], HIGH);
    } else {
      // Set direction pin LOW for reverse calibration
      digitalWrite(dirPins[i], LOW);
    }
  }

  // Initialize completion status based on which axes have motion commands
  for (int i = 0; i < numJoints; i++) {
    // Set complete if joint was not sent a limit value
    // Axes with 0 steps are not part of this calibration
    if (steps[i] == 0) {
      // Mark as complete (no motion for this axis)
      complete[i] = 1;
    }
  }

  //DRIVE MOTORS FOR CALIBRATION
  // Main calibration loop - drives motors until all limits are hit
  int DriveLimInProc = 1;
  // Continue until all joints complete or emergency stop activated
  while (DriveLimInProc == 1 && estopActive == false) {

    // Process each joint independently
    for (int i = 0; i < numJoints; i++) {
      // Evaluate each joint
      // Read current state of limit switch
      curState[i] = digitalRead(calPins[i]);

      // Debounced: set complete only if HIGH is stable for DEBOUNCE_US
      // Check if switch is in expected state
      if (curState[i] == limitSensor[i]) {
        // Record time when switch first detected
        if (firstHighUs[i] == 0) firstHighUs[i] = micros();
        // Check if stable for required debounce time
        if ((micros() - firstHighUs[i]) >= DEBOUNCE_US) {
          // Mark joint as completed (limit detected)
          complete[i] = 1;
        }
      } else {
        // reset if it ever goes LOW again
        // Reset debounce timer if signal is unstable
        firstHighUs[i] = 0;  // reset if it ever goes LOW again
      }

      // Step the motor if not complete and curState is LOW
      // Execute motor step if limit not yet hit and steps remain
      if (stepsDone[i] < steps[i] && complete[i] == 0) {
        // Pulse step pin HIGH
        digitalWrite(stepPins[i], HIGH);
        // Allow step width timing
        delayMicroseconds(calcStepGap);
        // Return step pin LOW to complete pulse
        digitalWrite(stepPins[i], LOW);
        // Increment step counter
        stepsDone[i]++;
      } else if (stepsDone[i] >= steps[i]) {
        // Steps exceeded, sensor never triggered – consider it failed
        // Max steps reached without hitting limit - mark as complete anyway
        complete[i] = 1;
      }
    }

    // Check if all joints are complete
    // Scan all axes to see if all are done homing
    int allComplete = 1;
    // Loop through each joint
    for (int i = 0; i < numJoints; i++) {
      // Check if this joint is not complete
      if (complete[i] == 0) {
        // Still have incomplete axes
        allComplete = 0;
        // Stop checking - at least one axis not done
        break;
      }
    }

    // Exit main loop if all axes are complete
    if (allComplete == 1) {
      // Set exit condition for main loop
      DriveLimInProc = 0;
    }

    // Delay before restarting the loop
    // Brief pause between step pulse iterations
    delayMicroseconds(100);
  }
}

void backOff(uint8_t J1req, uint8_t J2req, uint8_t J3req, uint8_t J4req, uint8_t J5req,
             uint8_t J6req, uint8_t J7req, uint8_t J8req, uint8_t J9req) {

  // SET DIRECTIONS
  digitalWrite(J1dirPin, (J1CalDir == J1MotDir) ? LOW : HIGH);
  digitalWrite(J2dirPin, (J2CalDir == J2MotDir) ? LOW : HIGH);
  digitalWrite(J3dirPin, (J3CalDir == J3MotDir) ? LOW : HIGH);
  digitalWrite(J4dirPin, (J4CalDir == J4MotDir) ? LOW : HIGH);
  digitalWrite(J5dirPin, (J5CalDir == J5MotDir) ? LOW : HIGH);
  digitalWrite(J6dirPin, (J6CalDir == J6MotDir) ? LOW : HIGH);
  digitalWrite(J7dirPin, (J7CalDir == J7MotDir) ? LOW : HIGH);
  digitalWrite(J8dirPin, (J8CalDir == J8MotDir) ? LOW : HIGH);
  digitalWrite(J9dirPin, (J9CalDir == J9MotDir) ? LOW : HIGH);

  auto pulseStep = [](uint8_t pin) {
    digitalWrite(pin, LOW);
    delayMicroseconds(5);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
  };

  int BacOff = 0;
  while (BacOff <= 250) {

    if (J1req == 1) pulseStep(J1stepPin);
    if (J2req == 1) pulseStep(J2stepPin);
    if (J3req == 1) pulseStep(J3stepPin);
    if (J4req == 1) pulseStep(J4stepPin);
    if (J5req == 1) pulseStep(J5stepPin);
    if (J6req == 1) pulseStep(J6stepPin);
    if (J7req == 1) pulseStep(J7stepPin);
    if (J8req == 1) pulseStep(J8stepPin);
    if (J9req == 1) pulseStep(J9stepPin);

    BacOff++;  // same effect as: BacOff = ++BacOff;
    delayMicroseconds(5000);
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENCODER & COLLISION RESET
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void resetEncoders()
//
// Synchronizes encoder readings with internal step counters and clears collision flags.
// Called before motion begins to establish baseline for collision detection.
// Resets all joint collision flags to prepare for next motion sequence.
//
void resetEncoders() {

  // Clear all collision detection flags for new motion command
  J1collisionTrue = 0;
  // Reset J2 collision flag
  J2collisionTrue = 0;
  // Reset J3 collision flag
  J3collisionTrue = 0;
  // Reset J4 collision flag
  J4collisionTrue = 0;
  // Reset J5 collision flag
  J5collisionTrue = 0;
  // Reset J6 collision flag
  J6collisionTrue = 0;
}
//Made by Justin Fauson
//Sets Master step to encoder position
void readEncoders() {
      J1MasterStep = J1EncoderPosition.read() / J1EncoderMultiplier;
      J2MasterStep = J2EncoderPosition.read() / J2EncoderMultiplier;
      J3MasterStep = J3EncoderPosition.read() / J3EncoderMultiplier;
      J4MasterStep = J4EncoderPosition.read() / J4EncoderMultiplier;
      J5MasterStep = J5EncoderPosition.read() / J5EncoderMultiplier;
      J6MasterStep = J6EncoderPosition.read() / J6EncoderMultiplier;
}
// ==================================================================================
// ENCODER FEEDBACK & COLLISION DETECTION
// ==================================================================================
// void checkEncoders()
//
// Reads all joint encoders and compares against commanded step positions to detect:
// - Collision/stall conditions when encoder lags beyond threshold (encOffset)
// - Motor slip or jamming
// - Mechanical failures
//
// In closed-loop mode (LoopMode=0), updates internal position tracking to match
// encoder readings for drift correction. Logs collision flags for error reporting.
//
void checkEncoders() {
  //read encoders
  // Read J1 encoder position and scale by multiplier
  J1EncSteps = J1EncoderPosition.read() / J1EncoderMultiplier;
  // Read J2 encoder position and scale by multiplier
  J2EncSteps = J2EncoderPosition.read() / J2EncoderMultiplier;
  // Read J3 encoder position and scale by multiplier
  J3EncSteps = J3EncoderPosition.read() / J3EncoderMultiplier;
  // Read J4 encoder position and scale by multiplier
  J4EncSteps = J4EncoderPosition.read() / J4EncoderMultiplier;
  // Read J5 encoder position and scale by multiplier
  J5EncSteps = J5EncoderPosition.read() / J5EncoderMultiplier;
  // Read J6 encoder position and scale by multiplier
  J6EncSteps = J6EncoderPosition.read() / J6EncoderMultiplier;
  //Check for collision and update position based on encoder
  // Check if J1 encoder position differs significantly from commanded position
  if (abs((J1EncSteps - J1MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision/stall
    if (J1LoopMode == 0) {
      // Set collision flag for J1
      J1collisionTrue = 1;
      // Update step counter to match actual encoder position
      J1MasterStep = J1EncoderPosition.read() / J1EncoderMultiplier;
    }
  }
  // Check if J2 encoder position differs significantly from commanded position
  if (abs((J2EncSteps - J2MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision
    if (J2LoopMode == 0) {
      // Set collision flag for J2
      J2collisionTrue = 1;
      // Update step counter to match actual encoder position
      J2MasterStep = J2EncoderPosition.read() / J2EncoderMultiplier;
    }
  }
  // Check if J3 encoder position differs significantly from commanded position
  if (abs((J3EncSteps - J3MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision
    if (J3LoopMode == 0) {
      // Set collision flag for J3
      J3collisionTrue = 1;
      // Update step counter to match actual encoder position
      J3MasterStep = J3EncoderPosition.read() / J3EncoderMultiplier;
    }
  }
  // Check if J4 encoder position differs significantly from commanded position
  if (abs((J4EncSteps - J4MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision
    if (J4LoopMode == 0) {
      // Set collision flag for J4
      J4collisionTrue = 1;
      // Update step counter to match actual encoder position
      J4MasterStep = J4EncoderPosition.read() / J4EncoderMultiplier;
    }
  }
  // Check if J5 encoder position differs significantly from commanded position
  if (abs((J5EncSteps - J5MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision
    if (J5LoopMode == 0) {
      // Set collision flag for J5
      J5collisionTrue = 1;
      // Update step counter to match actual encoder position
      J5MasterStep = J5EncoderPosition.read() / J5EncoderMultiplier;
    }
  }
  // Check if J6 encoder position differs significantly from commanded position
  if (abs((J6EncSteps - J6MasterStep)) >= encOffset) {
    // If in closed-loop mode, this indicates a collision
    if (J6LoopMode == 0) {
      // Set collision flag for J6
      J6collisionTrue = 1;
      // Update step counter to match actual encoder position
      J6MasterStep = J6EncoderPosition.read() / J6EncoderMultiplier;
    }
  }

  // Calculate total collision count from all joints
  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  // Check if any collisions were detected
  if (TotalCollision > 0) {
    // Build error code string showing which joints have collisions
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DRIVE MOTORS J - JOINT SPACE STEPPER MOTOR CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// driveMotorsJ(int J1-J9step, dirs, String SpeedType, float SpeedVal, ACCspd, DCCspd, ACCramp)
//
// Low-level stepper motor driver for joint-space motion. Uses linear interpolation algorithm
// to synchronize 9 independent stepper motors for coordinated motion. Supports:
// - Acceleration/deceleration profiling with configurable ramp times
// - Trapezoid velocity profile (ramp up, constant speed, ramp down)
// - Per-joint step target and direction control
// - Speed limits with adaptive delay calculation
//
// The algorithm distributes steps across motors using linear interpolation to ensure
// smooth synchronized motion (Bresenham-style line drawing algorithm).
//
void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step,
                  int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir,
                  String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {
  // Array of steps and directions
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // Array of step and direction pins
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };

  // Initialize step monitors
  int stepMonitors[9] = { J1MasterStep, J2MasterStep, J3MasterStep, J4MasterStep, J5MasterStep, J6MasterStep, J7MasterStep, J8MasterStep, J9MasterStep };

  int HighStep = steps[0];
  int Jactive = 0;

  // FIND HIGHEST STEP
  for (int i = 1; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
    if (steps[i] >= 1) {
      active[i] = 1;
      Jactive++;
    }
  }

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  /////CALC SPEEDS//////
  float calcStepGap;  // cruise delay (µs between highStep ticks)
  float speedSP;      // target total time in µs for the move
  float delay;

  // DETERMINE STEPS
  float ACCStep = HighStep * (ACCspd / 100.0f);
  float DCCStep = HighStep * (DCCspd / 100.0f);
  float NORStep = HighStep - ACCStep - DCCStep;

  // SET SPEED FOR SECONDS OR MM PER SEC
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000.0f) * 1.0f;
  } else if (SpeedType == "m") {
    float lineDist = pow(pow(xyzuvw_In[0] - xyzuvw_Out[0], 2) + pow(xyzuvw_In[1] - xyzuvw_Out[1], 2) + pow(xyzuvw_In[2] - xyzuvw_Out[2], 2), 0.5f);
    speedSP = ((lineDist / SpeedVal) * 1000000.0f) * 1.0f;
  }

  // fixed ramp factors (start/end slower than cruise)
  if (ACCramp < 10) {
    ACCramp = 10;
  }
  const float k_acc = ACCramp / 10;
  const float k_dec = ACCramp / 10;

  if (SpeedType == "s" || SpeedType == "m") {
    // Solve cruise delay so total time matches speedSP.
    //
    // Total time T for a trapezoid (linear accel/decel):
    // T = ACCStep * (start+cruise)/2 + NORStep * (cruise) + DCCStep * (cruise+end)/2
    // Let start = k_acc * cruise, end = k_dec * cruise => solve for cruise:
    //
    // T = cruise * [ NORStep + (ACCStep*(1+k_acc) + DCCStep*(1+k_dec))/2 ]
    //
    float denom = NORStep + (ACCStep * (1.0f + k_acc) + DCCStep * (1.0f + k_dec)) * 0.5f;

    if (denom <= 0.0f) {
      // Fallback to constant speed if accel+decel consume everything
      calcStepGap = speedSP / max(HighStep, 1.0f);
    } else {
      calcStepGap = speedSP / denom;
    }

    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  } else if (SpeedType == "p") {
    // Percentage mode unchanged
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0f);
  }

  // With cruise known, define start/end delays and per-step increments
  float startDelay = calcStepGap * k_acc;  // slower than cruise
  float endDelay = calcStepGap * k_dec;    // slower than cruise

  // Linear ramp decrements/increments per step
  float calcACCstepInc = (ACCStep > 0.0f) ? (startDelay - calcStepGap) / ACCStep : 0.0f;  // subtract each step
  float calcDCCstepInc = (DCCStep > 0.0f) ? (endDelay - calcStepGap) / DCCStep : 0.0f;    // add each step

  // Start at the slow end of accel (or keep rounding behavior)
  float calcACCstartDel = startDelay;
  float curDelay = (rndTrue == true) ? rndSpeed : calcACCstartDel;
  rndTrue = false;

  ///// DRIVE MOTORS /////
  unsigned long moveStart = micros();
  int highStepCur = 0;

  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && estopActive == false) {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      // During accel, move from startDelay down to cruise
      curDelay -= calcACCstepInc;  // since calcACCstepInc = (start - cruise)/ACCStep > 0
    } else if (highStepCur >= (HighStep - DCCStep)) {
      // During decel, move from cruise up to endDelay
      curDelay += calcDCCstepInc;  // since calcDCCstepInc = (end - cruise)/DCCStep > 0
    } else {
      curDelay = calcStepGap;  // cruise
    }

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? (HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i]))) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = SE_2[i] + 1;
        }

        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = SE_1[i] + 1;
          }

          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;

            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;

              if (dirs[i] == 0) {
                stepMonitors[i]--;
              } else {
                stepMonitors[i]++;
              }
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }

    delay = curDelay - disDelayCur;
    if (delay < minSpeedDelay) {
      delay = minSpeedDelay;
    }
    delayMicroseconds(delay);
  }
  unsigned long moveEnd = micros();
  float elapsedSeconds = (moveEnd - moveStart) / 1000000.0f;
  //debug = String(elapsedSeconds);

  // Set rounding speed to last move speed
  rndSpeed = curDelay;

  // Update the original step monitor variables
  J1MasterStep = stepMonitors[0];
  J2MasterStep = stepMonitors[1];
  J3MasterStep = stepMonitors[2];
  J4MasterStep = stepMonitors[3];
  J5MasterStep = stepMonitors[4];
  J6MasterStep = stepMonitors[5];
  J7MasterStep = stepMonitors[6];
  J8MasterStep = stepMonitors[7];
  J9MasterStep = stepMonitors[8];
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DRIVE MOTORS G - GCODE STEPPER MOTOR CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// driveMotorsG(int J1-J9step, dirs, String SpeedType, float SpeedVal, ACCspd, DCCspd, ACCramp)
//
// Stepper motor driver for G-code motion execution. Similar to driveMotorsJ but optimized
// for G-code command interpretation with speed profile based on feed rate (mm/min).
// Supports coordinated multi-axis motion with acceleration profiling.
//
void driveMotorsG(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {
  int steps[] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int motDirs[] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepPins[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepMonitors[] = { J1MasterStep, J2MasterStep, J3MasterStep, J4MasterStep, J5MasterStep, J6MasterStep, J7MasterStep, J8MasterStep, J9MasterStep };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }

  // FIND ACTIVE JOINTS
  int Jactive = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      Jactive++;
    }
  }

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  int highStepCur = 0;
  float curDelay = 0;
  float speedSP;
  float moveDist;

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  ///// CALC SPEEDS /////
  float calcStepGap;
  speedViolation = "0";  // Reset speed violation flag

  // Set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * 1.2;
    calcStepGap = speedSP / HighStep;
  } else if (SpeedType == "m") {
    if (SpeedVal >= maxMMperSec) {
      SpeedVal = maxMMperSec;
      speedViolation = "1";
    }
    SpeedVal = ((SpeedVal / maxMMperSec) * 100);
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  } else if (SpeedType == "p") {
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  }

  // Ensure calcStepGap is not less than minSpeedDelay
  if (calcStepGap <= minSpeedDelay) {
    calcStepGap = minSpeedDelay;
    speedViolation = "1";
  }

  ///// DRIVE MOTORS /////
  while ((cur[0] != steps[0] || cur[1] != steps[1] || cur[2] != steps[2] || cur[3] != steps[3] || cur[4] != steps[4] || cur[5] != steps[5] || cur[6] != steps[6] || cur[7] != steps[7] || cur[8] != steps[8]) && estopActive == false) {
    curDelay = calcStepGap;

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = LO_1[i] > 0 ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = SE_1[i] > 0 ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = LO_2[i] > 0 ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) SE_2cur[i] = SE_2[i] + 1;
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) SE_1cur[i] = SE_1[i] + 1;
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    delayMicroseconds(curDelay - disDelayCur);
  }

  // set rounding speed to last move speed
  rndSpeed = curDelay;

  // assign the updated values back to the original step monitors
  J1MasterStep = stepMonitors[0];
  J2MasterStep = stepMonitors[1];
  J3MasterStep = stepMonitors[2];
  J4MasterStep = stepMonitors[3];
  J5MasterStep = stepMonitors[4];
  J6MasterStep = stepMonitors[5];
  J7MasterStep = stepMonitors[6];
  J8MasterStep = stepMonitors[7];
  J9MasterStep = stepMonitors[8];
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DRIVE MOTORS L - LINEAR MOTION STEPPER CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// driveMotorsL(int J1-J9step, dirs, float curDelay)
//
// Fixed-speed stepper motor driver for linear motion execution in lookahead mode.
// Uses constant delay (step interval) for uniform motion speed. Used in spline mode
// and continuous motion sequences where trajectory timing is pre-calculated.
//
void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, float curDelay) {
  // Array of steps, directions, pins, motor directions, and step counters
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepMonitors[9] = { J1MasterStep, J2MasterStep, J3MasterStep, J4MasterStep, J5MasterStep, J6MasterStep, J7MasterStep, J8MasterStep, J9MasterStep };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }

  // FIND ACTIVE JOINTS
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      active[i] = 1;
    }
  }

  // Process lookahead
  if (splineTrue) {
    processSerial();
  }

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  int highStepCur = 0;

  // DRIVE MOTORS
  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && !estopActive) {
    float distDelay = 30;
    float disDelayCur = 0;

    // Process lookahead
    if (splineTrue) {
      processSerial();
    }

    // Iterate through each joint
    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = 1;
        }
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = 1;
          }
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    delayMicroseconds(curDelay - disDelayCur);
  }

  // Assign the updated values back to the original step monitors
  J1MasterStep = stepMonitors[0];
  J2MasterStep = stepMonitors[1];
  J3MasterStep = stepMonitors[2];
  J4MasterStep = stepMonitors[3];
  J5MasterStep = stepMonitors[4];
  J6MasterStep = stepMonitors[5];
  J7MasterStep = stepMonitors[6];
  J8MasterStep = stepMonitors[7];
  J9MasterStep = stepMonitors[8];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOVE J - JOINT MOTION PLANNING AND EXECUTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// moveJ(String inData, bool response, bool precalc, bool simspeed)
//
// Performs inverse kinematics (IK) on target Cartesian position and executes joint motion
// to reach the target. Handles motion control parameters including acceleration, deceleration,
// speed limits, and corner rounding. Supports multiple kinematics solutions via wrist config.
//
// Parameters:
//   inData - Movement command string containing: X,Y,Z,Rx,Ry,Rz (Cartesian), J7-J9 (external),
//            S (speed type/value), Ac (accel), Dc (decel), Rm (ramp), Rnd (rounding), W (wrist config)
//   response - If true, send position response to host after motion complete
//   precalc - If true, use pre-calculated joint estimate for IK convergence
//   simspeed - If true, simulate G-code speed profile instead of direct speed value
//
void moveJ(String inData, bool response, bool precalc, bool simspeed) {
  
  int J1dir;
  int J2dir;
  int J3dir;
  int J4dir;
  int J5dir;
  int J6dir;
  int J7dir;
  int J8dir;
  int J9dir;

  int J1axisFault = 0;
  int J2axisFault = 0;
  int J3axisFault = 0;
  int J4axisFault = 0;
  int J5axisFault = 0;
  int J6axisFault = 0;
  int J7axisFault = 0;
  int J8axisFault = 0;
  int J9axisFault = 0;
  int TotalAxisFault = 0;

  int xStart = inData.indexOf("X");
  int yStart = inData.indexOf("Y");
  int zStart = inData.indexOf("Z");
  int rzStart = inData.indexOf("Rz");
  int ryStart = inData.indexOf("Ry");
  int rxStart = inData.indexOf("Rx");
  int J7Start = inData.indexOf("J7");
  int J8Start = inData.indexOf("J8");
  int J9Start = inData.indexOf("J9");
  int SPstart = inData.indexOf("S");
  int AcStart = inData.indexOf("Ac");
  int DcStart = inData.indexOf("Dc");
  int RmStart = inData.indexOf("Rm");
  int RndStart = inData.indexOf("Rnd");
  int WristConStart = inData.indexOf("W");
  int LoopModeStart = inData.indexOf("Lm");

  xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
  xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
  xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
  xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
  xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
  xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
  J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
  J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
  J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

  String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
  float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
  float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
  float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
  float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
  float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
  WristCon = inData.substring(WristConStart + 1, LoopModeStart);
  String LoopMode = inData.substring(LoopModeStart + 2);
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5).toInt();


  SolveInverseKinematics();

  //calc destination motor steps
  int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
  int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
  int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
  int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
  int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
  int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;
  int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
  int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
  int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;

  if (precalc) {
    J1MasterStep = J1FutureMasterStep;
    J2MasterStep = J2FutureMasterStep;
    J3MasterStep = J3FutureMasterStep;
    J4MasterStep = J4FutureMasterStep;
    J5MasterStep = J5FutureMasterStep;
    J6MasterStep = J6FutureMasterStep;
    J7MasterStep = J7futStepM;
    J8MasterStep = J8futStepM;
    J9MasterStep = J9futStepM;
  }

  else {
    //calc delta from current to destination
    int J1StepDelta = J1MasterStep - J1FutureMasterStep;
    int J2StepDelta = J2MasterStep - J2FutureMasterStep;
    int J3StepDelta = J3MasterStep - J3FutureMasterStep;
    int J4StepDelta = J4MasterStep - J4FutureMasterStep;
    int J5StepDelta = J5MasterStep - J5FutureMasterStep;
    int J6StepDelta = J6MasterStep - J6FutureMasterStep;
    int J7StepDelta = J7MasterStep - J7futStepM;
    int J8StepDelta = J8MasterStep - J8futStepM;
    int J9StepDelta = J9MasterStep - J9futStepM;

    //determine motor directions
    J1dir = (J1StepDelta <= 0) ? 1 : 0;
    J2dir = (J2StepDelta <= 0) ? 1 : 0;
    J3dir = (J3StepDelta <= 0) ? 1 : 0;
    J4dir = (J4StepDelta <= 0) ? 1 : 0;
    J5dir = (J5StepDelta <= 0) ? 1 : 0;
    J6dir = (J6StepDelta <= 0) ? 1 : 0;
    J7dir = (J7StepDelta <= 0) ? 1 : 0;
    J8dir = (J8StepDelta <= 0) ? 1 : 0;
    J9dir = (J9StepDelta <= 0) ? 1 : 0;

    // Arrays for joint properties
    int dir[numJoints] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
    int StepM[numJoints] = { J1MasterStep, J2MasterStep, J3MasterStep, J4MasterStep, J5MasterStep, J6MasterStep, J7MasterStep, J8MasterStep, J9MasterStep };
    int stepDif[numJoints] = { J1StepDelta, J2StepDelta, J3StepDelta, J4StepDelta, J5StepDelta, J6StepDelta, J7StepDelta, J8StepDelta, J9StepDelta };
    int StepLim[numJoints] = { J1StepRange, J2StepRange, J3StepRange, J4StepRange, J5StepRange, J6StepRange, J7StepRange, J8StepRange, J9StepRange };
    int axisFault[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Loop to check axis limits and set faults
    for (int i = 0; i < numJoints; ++i) {
      if ((dir[i] == 1 && (StepM[i] + stepDif[i] > StepLim[i])) || (dir[i] == 0 && (StepM[i] - stepDif[i] < 0))) {
        axisFault[i] = 1;
      }
    }

    // Assign fault values back to individual variables
    J1axisFault = axisFault[0];
    J2axisFault = axisFault[1];
    J3axisFault = axisFault[2];
    J4axisFault = axisFault[3];
    J5axisFault = axisFault[4];
    J6axisFault = axisFault[5];
    J7axisFault = axisFault[6];
    J8axisFault = axisFault[7];
    J9axisFault = axisFault[8];

    // Calculate total axis fault
    TotalAxisFault = 0;
    for (int i = 0; i < numJoints; ++i) {
      TotalAxisFault += axisFault[i];
    }

    //send move command if no axis limit error
    if (TotalAxisFault == 0 && KinematicError == 0) {
      resetEncoders();
      if (simspeed) {
        driveMotorsG(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      } else {
        driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      }
      checkEncoders();
      if (response == true) {
        sendRobotPos();
      }
    } else if (KinematicError == 1) {
      Alarm = "ER";
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
    } else {
      Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
    }

    inData = "";  // Clear recieved buffer
                  ////////MOVE COMPLETE///////////
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMUNICATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int32_t modbusQuerry(String inData, int function) {
  int32_t result;
  int32_t response;
  int32_t response2;
  int slaveIdIndex = inData.indexOf('A');
  int MBaddressIndex = inData.indexOf('B');
  int MBvalIndex = inData.indexOf('C');
  int SlaveID = inData.substring(slaveIdIndex + 1, MBaddressIndex).toInt();
  int MBaddress = inData.substring(MBaddressIndex + 1, MBvalIndex).toInt();
  int MBval = inData.substring(MBvalIndex + 1).toInt();
  node = ModbusMaster();
  node.begin(SlaveID, Serial8);

  if (function == 1) {
    result = node.readCoils(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 2) {
    result = node.readDiscreteInputs(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 3) {
    result = node.readHoldingRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 4) {
    result = node.readInputRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 15) {
    result = node.writeSingleCoil(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 6) {
    result = node.writeSingleRegister(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else {
    response = -1;
    return response;
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SERIAL COMMUNICATION & COMMAND BUFFERING
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void processSerial()
//
// Reads incoming serial data and buffers commands for execution. Implements:
// - Command queue with 3-level lookahead buffer (cmdBuffer1/2/3)
// - Spline mode detection and response timing optimization
// - Pre-processing of consecutive motion commands for lookahead trajectory planning
//
// In spline mode, automatically reads ahead to populate secondary buffer for smooth
// trajectory generation without interruption between moves.
//
void processSerial() {
  // Check if serial data is available and buffer 3 is not full
  if (Serial.available() > 0 and cmdBuffer3 == "") {
    // Read single character from serial input
    char recieved = Serial.read();
    // Append character to received data accumulator
    recData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n') {
      //place data in last position
      // Store complete command in third buffer position
      cmdBuffer3 = recData;
      //determine if move command
      // Create copy of received data for command type detection
      recData.trim();
      // Extract 2-character command code from string
      String procCMDtype = recData.substring(0, 2);
      // Check if spline sequence is ending
      if (procCMDtype == "SS") {
        // Disable spline mode
        splineTrue = false;
        // Mark that end-of-spline has been received
        splineEndReceived = true;
      }
      // Handle spline lookahead mode command response
      if (splineTrue == true) {
        // Check if first move has been started
        if (moveSequence == "") {
          // Mark that first motion command is active
          moveSequence = "firsMoveActive";
        }
        //close serial so next command can be read in
        // Check if no alarm condition exists
        if (Alarm == "0") {
          // Send position without clearing speed violation flag (for continuous updates)
          sendRobotPosSpline();
        } else {
          // Send alarm code instead of position
          Serial.println(Alarm);
          // Clear alarm for next command
          Alarm = "0";
        }
      }

      // Clear recieved buffer
      recData = "";  // Clear recieved buffer

      // Shift command buffers to process next command
      shiftCMDarray();

      //if second position is empty and first move command read in process second move ahead of time
      // Preprocess next motion command during spline execution for lookahead planning
      if (procCMDtype == "MS" and moveSequence == "firsMoveActive" and cmdBuffer2 == "" and cmdBuffer1 != "" and splineTrue == true) {
        // Mark that second motion has been preprocessed
        moveSequence = "secondMoveProcessed";
        // Wait for next command to be received
        while (cmdBuffer2 == "") {
          // Check for serial data availability
          if (Serial.available() > 0) {
            // Read character from serial port
            char recieved = Serial.read();
            // Append to received data
            recData += recieved;
            // Process when complete line received
            if (recieved == '\n') {
              // Store command in second buffer
              cmdBuffer2 = recData;
              // Check command type
              recData.trim();
              // Extract command code
              procCMDtype = recData.substring(0, 2);
              // Send position update if motion command
              if (procCMDtype == "MS") {
                // Allow time for processing
                delay(5);
                // Check for error conditions
                if (Alarm == "0") {
                  // Send position for spline lookahead
                  sendRobotPosSpline();
                } else {
                  // Send alarm condition
                  Serial.println(Alarm);
                  // Clear alarm
                  Alarm = "0";
                }
              }
              // Clear recieved buffer
              recData = "";  // Clear recieved buffer
            }
          }
        }
      }
    }
  }
}


void shiftCMDarray() {
  // Check if first buffer is empty and there is a second command
  if (cmdBuffer1 == "") {
    // Promote second buffer to first position
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    // Clear second buffer
    cmdBuffer2 = "";
  }
  // Check if second buffer is empty and there is a third command
  if (cmdBuffer2 == "") {
    // Promote third buffer to second position
    //shift 3 to 2
    cmdBuffer2 = cmdBuffer3;
    // Clear third buffer
    cmdBuffer3 = "";
  }
  // Check again if first buffer is still empty after shifting
  if (cmdBuffer1 == "") {
    // Promote second to first if needed
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    // Clear second buffer
    cmdBuffer2 = "";
  }
}


void EstopProg() {
  // Set emergency stop active flag to halt all motion
  estopActive = true;
  // Set error flag code for E-stop
  flag = "EB";
  // Send current position and error status to host
  sendRobotPos();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // ==================================================================================
  // TEENSY INITIALIZATION - Runs once at startup to configure hardware and state
  // ==================================================================================

  // Initialize serial communications for main control interface
  Serial.begin(9600);  // Main serial for host communication (USB/debug)
  // Initialize Modbus serial communication for external devices
  Serial8.begin(38400);  // Modbus serial interface (pins 34 and 35)
  // Note: No Serial output before this line to maintain timing/initialization order
  
  // Load persistent configuration from EEPROM
  load_debug_from_eeprom();      // Restore debug mode setting from EEPROM
  // Restore robot model, version, serial number info from persistent storage
  load_robot_id_from_eeprom();   // Restore robot model, version, serial number info

  // Initialize Modbus master node for industrial device communication (slave ID 1)
  node.begin(1, Serial8);

  // ==================================================================================
  // CONFIGURE STEPPER MOTOR CONTROL PINS - All 9 joints (step/direction pairs)
  // ==================================================================================
  // Set all motor step and direction pins as outputs for PWM control
  pinMode(J1stepPin, OUTPUT);
  // Configure J1 direction pin for motor control
  pinMode(J1dirPin, OUTPUT);
  // Configure J2 step pulse pin for stepper motor timing
  pinMode(J2stepPin, OUTPUT);
  // Set J2 direction control pin as output
  pinMode(J2dirPin, OUTPUT);
  // Configure J3 step pulse pin for motor stepping
  pinMode(J3stepPin, OUTPUT);
  // Set J3 direction pin for motor direction control
  pinMode(J3dirPin, OUTPUT);
  // Configure J4 step pin for motor pulse generation
  pinMode(J4stepPin, OUTPUT);
  // Set J4 direction control pin as output
  pinMode(J4dirPin, OUTPUT);
  // Configure J5 step pin for stepper motor control
  pinMode(J5stepPin, OUTPUT);
  // Set J5 direction pin for motor direction
  pinMode(J5dirPin, OUTPUT);
  // Configure J6 step pulse pin for motor timing
  pinMode(J6stepPin, OUTPUT);
  // Set J6 direction control pin as output
  pinMode(J6dirPin, OUTPUT);
  // Configure J7 step pin for external axis control
  pinMode(J7stepPin, OUTPUT);
  // Set J7 direction pin for axis direction
  pinMode(J7dirPin, OUTPUT);
  // Configure J8 step pin for external linear axis
  pinMode(J8stepPin, OUTPUT);
  // Set J8 direction pin for linear motion control
  pinMode(J8dirPin, OUTPUT);
  // Configure J9 step pin for external axis control
  pinMode(J9stepPin, OUTPUT);
  // Set J9 direction pin for axis direction
  pinMode(J9dirPin, OUTPUT);

  // ==================================================================================
  // CONFIGURE LIMIT/HOME SWITCH INPUTS - Calibration switches for all 9 joints
  // ==================================================================================
  // Set limit switch pins as inputs for detecting home/limit positions
  pinMode(J1calPin, INPUT);
  // Configure J2 home/limit switch input
  pinMode(J2calPin, INPUT);
  // Configure J3 home/limit switch input
  pinMode(J3calPin, INPUT);
  // Configure J4 home/limit switch input
  pinMode(J4calPin, INPUT);
  // Configure J5 home/limit switch input
  pinMode(J5calPin, INPUT);
  // Configure J6 home/limit switch input
  pinMode(J6calPin, INPUT);
  // Configure J7 home/limit switch input
  pinMode(J7calPin, INPUT);
  // Configure J8 home/limit switch input
  pinMode(J8calPin, INPUT);
  // Configure J9 home/limit switch input
  pinMode(J9calPin, INPUT);

  // ==================================================================================
  // CONFIGURE EMERGENCY STOP BUTTON
  // ==================================================================================
  // Configure E-stop as input with pull-up resistor (active LOW logic)
  pinMode(EstopPin, INPUT_PULLUP);
  // Attach interrupt handler to trigger on LOW signal (button pressed)
  attachInterrupt(digitalPinToInterrupt(EstopPin), EstopProg, LOW);

  // ==================================================================================
  // SET INITIAL MOTOR STATES
  // ==================================================================================
  // Set all step pins HIGH (inactive state) - motors step on HIGH->LOW transition
  digitalWrite(J1stepPin, HIGH);
  // Set J2 step pin to high (inactive)
  digitalWrite(J2stepPin, HIGH);
  // Set J3 step pin to high (inactive)
  digitalWrite(J3stepPin, HIGH);
  // Set J4 step pin to high (inactive)
  digitalWrite(J4stepPin, HIGH);
  // Set J5 step pin to high (inactive)
  digitalWrite(J5stepPin, HIGH);
  // Set J6 step pin to high (inactive)
  digitalWrite(J6stepPin, HIGH);
  // Set J7 step pin to high (inactive)
  digitalWrite(J7stepPin, HIGH);
  // Set J8 step pin to high (inactive)
  digitalWrite(J8stepPin, HIGH);
  // Set J9 step pin to high (inactive)
  digitalWrite(J9stepPin, HIGH);

  // ==================================================================================
  // INITIALIZE COMMAND AND MOTION STATE VARIABLES
  // ==================================================================================
  // Clear command buffers used for lookahead/spline motion sequencing
  cmdBuffer1 = "";
  // Clear second command buffer for pipelined motion commands
  cmdBuffer2 = "";
  // Clear third command buffer for lookahead planning
  cmdBuffer3 = "";
  
  // Reset motion control flags and identifiers
  moveSequence = "";            // Clear move sequence identifier
  // Clear status flag string for error reporting
  flag = "";                    // Clear status flag string
  // Disable corner rounding (not active on startup)
  rndTrue = false;              // Disable corner rounding (not active on startup)
  // Disable spline interpolation mode on startup
  splineTrue = false;           // Disable spline interpolation
  // Clear end-of-spline marker flag
  splineEndReceived = false;    // Clear end-of-spline marker
}

void loop() {
  // ==================================================================================
  // MAIN CONTROL LOOP - Continuous execution for command processing and motion control
  // ==================================================================================
  // This loop runs continuously and processes one command from the command buffer
  // per iteration. It handles serial communication, kinematics, motor control, and
  // motion profile generation (spline/lookahead).
  // ==================================================================================

  // ==================================================================================
  // SERIAL COMMUNICATION STAGE - Read incoming commands from host
  // ==================================================================================
  // Only read new serial data if not in the middle of a spline sequence
  if (splineEndReceived == false) {
    // Read incoming serial commands and populate command buffers
    processSerial();  // Read incoming serial commands and populate command buffers
  }
  if (J1LoopMode == 0 || J2LoopMode == 0 || J3LoopMode == 0 || J4LoopMode == 0 || J5LoopMode == 0 || J6LoopMode == 0) {
    closedLoopTrue = 1;
  }
  // ==================================================================================
  // COMMAND PROCESSING STAGE - Execute command from primary buffer
  // ==================================================================================
  // Only process if at least one command has been received and buffered
  if (cmdBuffer1 != "") {
    // Initialize processing state
    estopActive = false;               // Clear emergency stop flag
    // Load command from primary buffer
    inData = cmdBuffer1;               // Load command from primary buffer
    // Remove leading/trailing whitespace
    inData.trim();                     // Remove leading/trailing whitespace
    // Extract 2-letter function code from command
    String function = inData.substring(0, 2);  // Extract 2-letter function code
    // Remove function code from data string for parameter parsing
    inData = inData.substring(2);      // Remove function code from data string
    // Clear kinematic error flag for new motion command
    KinematicError = 0;                // Clear kinematic error flag
    // Clear debug message for next command
    debug = "";                        // Clear debug message

    // ==================================================================================
    // COMMAND DISPATCHER - Route to handler based on function code
    // ==================================================================================
    // Each command is identified by a 2-letter code followed by parameter data

    if (function == "HO") {
      // Debug output for received command
      DEBUG_PRINTLN("Debug - Received HO command");
      // Send system identification and status
      handle_hello_command();  // Send system identification and status
    }

    else if (function == "RB") {
      // System restart command - reboot Teensy controller
      Serial.println("System Restarting");
      // Perform software reset of Teensy 4.1
      reboot();
    }

    else if (function == "DB") {
      // DEBUG CONFIGURATION COMMAND - Enable/disable debug output and persistence
      // Parameters: [D]<0|1> = debug state, [P]<0|1> = enable persistence across reboots
      String help = "Command DB - Set Debug Parameters\n";
      // Add help text for debug state parameter
      help += "required [D] - Debug State 0/1 (off/on) Enables / Disabled Serial Debug Mode\n";
      // Add help text for persistence parameter
      help += "optional [P] - Persistence 0/1 Disable / Enable debug mode persist accross reboots\n\n";
      // Add example command formats
      help += "Example: DB[D]1[P]1 - Enabled Debug mode with persist\n";
      // Add alternative example
      help += "Example: DB[0] - Disable debug mode, don't change current persisted value\n\n";

      // Find start position of debug parameter in command string
      int debugStart = inData.indexOf("[D]");
      // Find start position of persistence parameter
      int persistStart = inData.indexOf("[P]", persistStart + 3);

      if (debugStart == -1) {
        // Debug parameter not found - invalid command format
        inData = "";
        // Clear command buffer
        cmdBuffer1 = "";  // Clear command buffer
        // Send help information to user
        Serial.println(help);
        // Return to main loop without processing
        return;
      }

      // Extract debug value from command string
      String debugValue = inData.substring(debugStart + 3, debugStart + 4);
      // Validate debug value is 0 or 1
      if (debugValue != "0" and debugValue != "1") {
        // Invalid value provided
        Serial.println("Valid values for debug are 0 and 1\n");
        // Display help information
        Serial.println(help);
        // Clear command data
        inData = "";
        // Clear command buffer
        cmdBuffer1 = "";  // Clear command buffer
        // Return to main loop
        return;
      }

      // Check if debug value is 0 (disable)
      if (debugValue == "0") {
        // Disable debug output
        DEBUG = false;
        // Print debug notification
        DEBUG_PRINTLN("Debug - Debugging Live Toggled Off");
      } else if (debugValue == "1") {
        // Enable debug output
        DEBUG = true;
        // Print debug notification
        DEBUG_PRINTLN("Debug - Debugging Live Toggled On");
      }

      // Check if persistence parameter was provided
      if (persistStart > 0) {
        // Extract persistence value from command
        String persistValue = inData.substring(persistStart + 3, persistStart + 4);
        // Validate persistence value is 0 or 1
        if (persistValue != "0" and persistValue != "1") {
          // Invalid value provided
          Serial.println("Valid values for persist are 0 and 1\n");
          // Display help information
          Serial.println(help);
          // Clear command data
          inData = "";
          // Clear command buffer
          cmdBuffer1 = "";  // Clear command buffer
          // Return to main loop
          return;
        } else {
          // Log persistence setting action
          DEBUG_PRINT("Setting Debug Persistence to: ");
          // Print the value being set
          DEBUG_PRINTLN(persistValue);

          // Check if enabling persistence
          if (persistValue == "1") {
            // Save debug enabled to EEPROM
            save_debug_to_eeprom(true);
          } else {
            // Save debug disabled to EEPROM
            save_debug_to_eeprom(false);
          }
        }
      }

      // Send success response
      Serial.println("Done");
      // Clear command data
      inData = "";
      // clear buffer
      cmdBuffer1 = "";  // clear buffer
      // Return to main loop
      return;

    }

    else if (function == "SR") {
      // SET ROBOT IDENTIFICATION COMMAND - Store hardware/software info to EEPROM
      // Parameters: [M]<model>[V]<version>[B]<board>[S]<serial>[A]<asset_tag>
      // Log received command with data for debugging
      DEBUG_PRINT("Debug - Received SR command with inData: ");
      // Print command parameters
      DEBUG_PRINTLN(inData);

      // Find position of model parameter marker in string
      int modelStart = inData.indexOf("[M]");
      // Find position of version parameter marker
      int versionStart = inData.indexOf("[V]", modelStart + 3);
      // Find position of driver board parameter marker
      int driverStart = inData.indexOf("[B]", versionStart + 3);
      // Find position of serial number parameter marker
      int serialStart = inData.indexOf("[S]", driverStart + 3);
      // Find position of asset tag parameter marker
      int assetStart = inData.indexOf("[A]", serialStart + 3);

      // Validate all required parameters are present
      if (modelStart == -1 || versionStart == -1 || driverStart == -1 || serialStart == -1 || assetStart == -1) {
        // Display error if format is incorrect
        Serial.println("Error: Invalid format (SR)");
        // Clear command data
        inData = "";
        // <-- drop the bad message
        cmdBuffer1 = "";  // <-- drop the bad message
        // Return to main loop
        return;
      }

      // Extract robot model string from command
      robot_model = inData.substring(modelStart + 3, versionStart);
      // Extract robot version string from command
      robot_version = inData.substring(versionStart + 3, driverStart);
      // Extract driver board string from command
      driver_board = inData.substring(driverStart + 3, serialStart);
      // Extract serial number string from command
      serial_number = inData.substring(serialStart + 3, assetStart);
      // Extract asset tag string from command
      asset_tag = inData.substring(assetStart + 3);

      // Debug output for extracted robot model
      DEBUG_PRINT("Debug - Robot Model extracted: ");
      // Print model value
      DEBUG_PRINTLN(robot_model);
      // Debug output for robot version
      DEBUG_PRINT("Debug - Robot Version extracted: ");
      // Print version value
      DEBUG_PRINTLN(robot_version);
      // Debug output for driver board
      DEBUG_PRINT("Debug - Driver Board extracted: ");
      // Print driver board value
      DEBUG_PRINTLN(driver_board);
      // Debug output for serial number
      DEBUG_PRINT("Debug - Serial Number: ");
      // Print serial number value
      DEBUG_PRINTLN(serial_number);
      // Debug output for asset tag
      DEBUG_PRINT("Debug - Asset Tag extracted: ");
      // Print asset tag value
      DEBUG_PRINTLN(asset_tag);

      // Call handler to save robot identification to EEPROM
      handle_set_robot_id_command(robot_model, robot_version, driver_board, serial_number, asset_tag);
    }

    // ==================================================================================
    // MODBUS COMMUNICATION COMMANDS - Industrial device control via RS-485
    // ==================================================================================

    else if (function == "BA") {
      // MODBUS: Read Holding Register (Function 03)
      // Reads single/multiple 16-bit holding registers from slave device
      int32_t result = modbusQuerry(inData, 3);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    else if (function == "BB") {
      // MODBUS: Read Coil Status (Function 01)
      // Reads single/multiple ON/OFF coil values from slave device
      int32_t result = modbusQuerry(inData, 1);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    else if (function == "BC") {
      // MODBUS: Read Input Status (Function 02)
      // Reads single/multiple ON/OFF input values from slave device
      int32_t result = modbusQuerry(inData, 2);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    else if (function == "BD") {
      // MODBUS: Read Input Register (Function 04)
      // Reads single/multiple 16-bit input registers from slave device
      int32_t result = modbusQuerry(inData, 4);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    else if (function == "BE") {
      // MODBUS: Write Multiple Coils (Function 15)
      // Writes ON/OFF coil values to slave device
      int32_t result = modbusQuerry(inData, 15);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    else if (function == "BF") {
      // MODBUS: Write Single Register (Function 06)
      // Writes single 16-bit register value to slave device
      int32_t result = modbusQuerry(inData, 6);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    else if (function == "MQ") {
      // MODBUS: Query Drive Position - Read absolute position counter
      // Reads holding register 0x1207 from drive (absolute position)
      uint8_t result;
      int16_t highRegister;

      // Modbus read
      result = node.readHoldingRegisters(0x1207, 2);

      if (result == node.ku8MBSuccess) {

        highRegister = node.getResponseBuffer(0);
        Serial.println(highRegister);

      } else {
        Serial.println("Modbus error: ");
        //Serial.println(result, HEX);
      }

      delay(1000);
    }

    else if (function == "HD") {
      // MODBUS: Home Drive Motor - Trigger homing sequence on drive
      // Sends control signals to drive for calibration/home position
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3
      uint16_t registerAddress2 = 0x020C;  // P0212 - DI2
      //uint16_t registerAddress = 0x1207;  // P1807 - absolute position counter
      uint16_t valueOn = 1;   // Value to write to the register
      uint16_t valueOff = 0;  // Value to write to the register

      // Write the value to the register
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOff);

      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        //Serial.println("Modbus Error: ");
        Serial.println(result, HEX);
      }

      delay(50);
    }

    else if (function == "RR") {
      // MODBUS: Reset Drive - Clear drive faults and reinitialize
      // Writes control registers to drive for reset sequence
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3 INPUT
      uint16_t registerAddress2 = 0x0203;  // P0203 - DI3 FUNCTION SELECTION

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;
      uint16_t homingMode = 33;
      uint16_t resetMode = 2;


      result = node.writeSingleRegister(registerAddress2, resetMode);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, homingMode);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

    else if (function == "FR") {
      // MODBUS: Fault Reset - Clear fault condition on drive
      // Writes fault reset command to drive register 0x0B01
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x0B01;  // P1101 - fault reset

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;

      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

    // ==================================================================================
    // MOTION CONTROL COMMANDS - Spline/Lookahead motion sequences
    // ==================================================================================

    else if (function == "SL") {
      // SPLINE START - Begin continuous spline motion sequence
      // Enables spline interpolation mode for smooth path following
      splineTrue = true;
      delay(5);
      Serial.print("SL");
      moveSequence = "";
      flag = "";
      rndTrue = false;
      splineEndReceived = false;
    }

    else if (function == "SS") {
      // SPLINE STOP - End spline motion sequence
      // Terminates spline mode and sends final robot position to host
      delay(5);
      sendRobotPos();
      splineTrue = false;
      splineEndReceived = false;
    }

    // ==================================================================================
    // SYSTEM & DIAGNOSTIC COMMANDS
    // ==================================================================================

    else if (function == "CL") {
      // CLOSE CONNECTION - Terminate serial communication
      // Safely closes serial port connection
      delay(5);
      Serial.end();
    }

    else if (function == "TL") {
      // TEST LIMIT SWITCHES - Read status of all home/limit switches
      // Returns 0/1 status for each joint's limit switch

      String J1calTest = "0";
      String J2calTest = "0";
      String J3calTest = "0";
      String J4calTest = "0";
      String J5calTest = "0";
      String J6calTest = "0";

      if (digitalRead(J1calPin) == HIGH) {
        J1calTest = "1";
      }
      if (digitalRead(J2calPin) == HIGH) {
        J2calTest = "1";
      }
      if (digitalRead(J3calPin) == HIGH) {
        J3calTest = "1";
      }
      if (digitalRead(J4calPin) == HIGH) {
        J4calTest = "1";
      }
      if (digitalRead(J5calPin) == HIGH) {
        J5calTest = "1";
      }
      if (digitalRead(J6calPin) == HIGH) {
        J6calTest = "1";
      }
      String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
      delay(5);
      Serial.println(TestLim);
    }


    else if (function == "SE") {
      // SET ENCODER VALUES - Reset all encoder counters to 1000
      // Used for encoder initialization or calibration
      J1EncoderPosition.write(1000);
      J2EncoderPosition.write(1000);
      J3EncoderPosition.write(1000);
      J4EncoderPosition.write(1000);
      J5EncoderPosition.write(1000);
      J6EncoderPosition.write(1000);
      delay(5);
      Serial.print("Done");
    }

    else if (function == "RE") {
      // READ ENCODER POSITIONS - Get current step count from all encoders
      // Returns encoder values for collision detection verification
      J1EncSteps = J1EncoderPosition.read();
      J2EncSteps = J2EncoderPosition.read();
      J3EncSteps = J3EncoderPosition.read();
      J4EncSteps = J4EncoderPosition.read();
      J5EncSteps = J5EncoderPosition.read();
      J6EncSteps = J6EncoderPosition.read();
      String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
      delay(5);
      Serial.println(Read);
    }

    else if (function == "RP") {
      // REQUEST POSITION - Query current robot joint angles
      // Returns current position or error state if alarm active
      //close serial so next command can be read in
      delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      } else {
        Serial.println(Alarm);
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    else if (function == "HM") {
      // MOVE HOME - Move all joints to zero (center) position
      // For debugging - performs motion to home position and verifies encoders

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;


      String SpeedType = "p";
      float SpeedVal = 25.0;
      float ACCspd = 10.0;
      float DCCspd = 10.0;
      float ACCramp = 20.0;

      CurrentJointAngle[0] = 0.00;
      CurrentJointAngle[1] = 0.00;
      CurrentJointAngle[2] = 0.00;
      CurrentJointAngle[3] = 0.00;
      CurrentJointAngle[4] = 0.00;
      CurrentJointAngle[5] = 0.00;


      //calc destination motor steps
      int J1FutureMasterStep = J1axisLimNeg * J1StepsPerDegree;
      int J2FutureMasterStep = J2axisLimNeg * J2StepDeg;
      int J3FutureMasterStep = J3axisLimNeg * J3StepDeg;
      int J4FutureMasterStep = J4axisLimNeg * J4StepDeg;
      int J5FutureMasterStep = J5axisLimNeg * J5StepDeg;
      int J6FutureMasterStep = J6axisLimNeg * J6StepDeg;

      //calc delta from current to destination
      int J1StepDelta = J1MasterStep - J1FutureMasterStep;
      int J2StepDelta = J2MasterStep - J2FutureMasterStep;
      int J3StepDelta = J3MasterStep - J3FutureMasterStep;
      int J4StepDelta = J4MasterStep - J4FutureMasterStep;
      int J5StepDelta = J5MasterStep - J5FutureMasterStep;
      int J6StepDelta = J6MasterStep - J6FutureMasterStep;
      int J7StepDelta = 0;
      int J8StepDelta = 0;
      int J9StepDelta = 0;

      //determine motor directions
      J1dir = (J1StepDelta <= 0) ? 1 : 0;
      J2dir = (J2StepDelta <= 0) ? 1 : 0;
      J3dir = (J3StepDelta <= 0) ? 1 : 0;
      J4dir = (J4StepDelta <= 0) ? 1 : 0;
      J5dir = (J5StepDelta <= 0) ? 1 : 0;
      J6dir = (J6StepDelta <= 0) ? 1 : 0;
      J7dir = 0;
      J8dir = 0;
      J9dir = 0;



      resetEncoders();//reset encoder faults
      driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      checkEncoders();//update encoder
      sendRobotPos();//send position to host
      delay(5);//might be to give time for serial to process idk
      Serial.println("Done");

    }


    else if (function == "CP") {
      // CORRECT POSITION - Synchronize internal position counters with encoder readings
      // Updates step counters to match encoder feedback for drift correction
      correctRobotPos();
    }

    else if (function == "UP") {
      // UPDATE PARAMETERS - Load all robot configuration parameters from host
      // Updates: tool frame, motor directions, calibration directions, axis limits,
      // steps/degree, encoder multipliers, and Denavit-Hartenberg parameters
      int TFxStart = inData.indexOf('A');
      int TFyStart = inData.indexOf('B');
      int TFzStart = inData.indexOf('C');
      int TFrzStart = inData.indexOf('D');
      int TFryStart = inData.indexOf('E');
      int TFrxStart = inData.indexOf('F');

      int J1motDirStart = inData.indexOf('G');
      int J2motDirStart = inData.indexOf('H');
      int J3motDirStart = inData.indexOf('I');
      int J4motDirStart = inData.indexOf('J');
      int J5motDirStart = inData.indexOf('K');
      int J6motDirStart = inData.indexOf('L');
      int J7motDirStart = inData.indexOf('M');
      int J8motDirStart = inData.indexOf('N');
      int J9motDirStart = inData.indexOf('O');

      int J1calDirStart = inData.indexOf('P');
      int J2calDirStart = inData.indexOf('Q');
      int J3calDirStart = inData.indexOf('R');
      int J4calDirStart = inData.indexOf('S');
      int J5calDirStart = inData.indexOf('T');
      int J6calDirStart = inData.indexOf('U');
      int J7calDirStart = inData.indexOf('V');
      int J8calDirStart = inData.indexOf('W');
      int J9calDirStart = inData.indexOf('X');

      int J1PosLimStart = inData.indexOf('Y');
      int J1NegLimStart = inData.indexOf('Z');
      int J2PosLimStart = inData.indexOf('a');
      int J2NegLimStart = inData.indexOf('b');
      int J3PosLimStart = inData.indexOf('c');
      int J3NegLimStart = inData.indexOf('d');
      int J4PosLimStart = inData.indexOf('e');
      int J4NegLimStart = inData.indexOf('f');
      int J5PosLimStart = inData.indexOf('g');
      int J5NegLimStart = inData.indexOf('h');
      int J6PosLimStart = inData.indexOf('i');
      int J6NegLimStart = inData.indexOf('j');

      int J1StepsPerDegreeStart = inData.indexOf('k');
      int J2StepDegStart = inData.indexOf('l');
      int J3StepDegStart = inData.indexOf('m');
      int J4StepDegStart = inData.indexOf('n');
      int J5StepDegStart = inData.indexOf('o');
      int J6StepDegStart = inData.indexOf('p');

      int J1EncoderMultiplierStart = inData.indexOf('q');
      int J2EncoderMultiplierStart = inData.indexOf('r');
      int J3EncoderMultiplierStart = inData.indexOf('s');
      int J4EncoderMultiplierStart = inData.indexOf('t');
      int J5EncoderMultiplierStart = inData.indexOf('u');
      int J6EncoderMultiplierStart = inData.indexOf('v');

      int J1tDHparStart = inData.indexOf('w');
      int J2tDHparStart = inData.indexOf('x');
      int J3tDHparStart = inData.indexOf('y');
      int J4tDHparStart = inData.indexOf('z');
      int J5tDHparStart = inData.indexOf('!');
      int J6tDHparStart = inData.indexOf('@');

      int J1uDHparStart = inData.indexOf('#');
      int J2uDHparStart = inData.indexOf('$');
      int J3uDHparStart = inData.indexOf('%');
      int J4uDHparStart = inData.indexOf('^');
      int J5uDHparStart = inData.indexOf('&');
      int J6uDHparStart = inData.indexOf('*');

      int J1dDHparStart = inData.indexOf('(');
      int J2dDHparStart = inData.indexOf(')');
      int J3dDHparStart = inData.indexOf('+');
      int J4dDHparStart = inData.indexOf('=');
      int J5dDHparStart = inData.indexOf(',');
      int J6dDHparStart = inData.indexOf('_');

      int J1aDHparStart = inData.indexOf('<');
      int J2aDHparStart = inData.indexOf('>');
      int J3aDHparStart = inData.indexOf('?');
      int J4aDHparStart = inData.indexOf('{');
      int J5aDHparStart = inData.indexOf('}');
      int J6aDHparStart = inData.indexOf('~');

      Robot_Kin_Tool[0] = inData.substring(TFxStart + 1, TFyStart).toFloat();
      Robot_Kin_Tool[1] = inData.substring(TFyStart + 1, TFzStart).toFloat();
      Robot_Kin_Tool[2] = inData.substring(TFzStart + 1, TFrzStart).toFloat();
      Robot_Kin_Tool[3] = inData.substring(TFrzStart + 1, TFryStart).toFloat() * M_PI / 180;
      Robot_Kin_Tool[4] = inData.substring(TFryStart + 1, TFrxStart).toFloat() * M_PI / 180;
      Robot_Kin_Tool[5] = inData.substring(TFrxStart + 1).toFloat() * M_PI / 180;
      J1MotDir = inData.substring(J1motDirStart + 1, J2motDirStart).toInt();
      J2MotDir = inData.substring(J2motDirStart + 1, J3motDirStart).toInt();
      J3MotDir = inData.substring(J3motDirStart + 1, J4motDirStart).toInt();
      J4MotDir = inData.substring(J4motDirStart + 1, J5motDirStart).toInt();
      J5MotDir = inData.substring(J5motDirStart + 1, J6motDirStart).toInt();
      J6MotDir = inData.substring(J6motDirStart + 1, J7motDirStart).toInt();
      J7MotDir = inData.substring(J7motDirStart + 1, J8motDirStart).toInt();
      J8MotDir = inData.substring(J8motDirStart + 1, J9motDirStart).toInt();
      J9MotDir = inData.substring(J9motDirStart + 1, J1calDirStart).toInt();
      J1CalDir = inData.substring(J1calDirStart + 1, J2calDirStart).toInt();
      J2CalDir = inData.substring(J2calDirStart + 1, J3calDirStart).toInt();
      J3CalDir = inData.substring(J3calDirStart + 1, J4calDirStart).toInt();
      J4CalDir = inData.substring(J4calDirStart + 1, J5calDirStart).toInt();
      J5CalDir = inData.substring(J5calDirStart + 1, J6calDirStart).toInt();
      J6CalDir = inData.substring(J6calDirStart + 1, J7calDirStart).toInt();
      J7CalDir = inData.substring(J7calDirStart + 1, J8calDirStart).toInt();
      J8CalDir = inData.substring(J8calDirStart + 1, J9calDirStart).toInt();
      J9CalDir = inData.substring(J9calDirStart + 1, J1PosLimStart).toInt();
      J1axisLimPos = inData.substring(J1PosLimStart + 1, J1NegLimStart).toFloat();
      J1axisLimNeg = inData.substring(J1NegLimStart + 1, J2PosLimStart).toFloat();
      J2axisLimPos = inData.substring(J2PosLimStart + 1, J2NegLimStart).toFloat();
      J2axisLimNeg = inData.substring(J2NegLimStart + 1, J3PosLimStart).toFloat();
      J3axisLimPos = inData.substring(J3PosLimStart + 1, J3NegLimStart).toFloat();
      J3axisLimNeg = inData.substring(J3NegLimStart + 1, J4PosLimStart).toFloat();
      J4axisLimPos = inData.substring(J4PosLimStart + 1, J4NegLimStart).toFloat();
      J4axisLimNeg = inData.substring(J4NegLimStart + 1, J5PosLimStart).toFloat();
      J5axisLimPos = inData.substring(J5PosLimStart + 1, J5NegLimStart).toFloat();
      J5axisLimNeg = inData.substring(J5NegLimStart + 1, J6PosLimStart).toFloat();
      J6axisLimPos = inData.substring(J6PosLimStart + 1, J6NegLimStart).toFloat();
      J6axisLimNeg = inData.substring(J6NegLimStart + 1, J1StepsPerDegreeStart).toFloat();

      J1StepsPerDegree = inData.substring(J1StepsPerDegreeStart + 1, J2StepDegStart).toFloat();
      J2StepDeg = inData.substring(J2StepDegStart + 1, J3StepDegStart).toFloat();
      J3StepDeg = inData.substring(J3StepDegStart + 1, J4StepDegStart).toFloat();
      J4StepDeg = inData.substring(J4StepDegStart + 1, J5StepDegStart).toFloat();
      J5StepDeg = inData.substring(J5StepDegStart + 1, J6StepDegStart).toFloat();
      J6StepDeg = inData.substring(J6StepDegStart + 1, J1EncoderMultiplierStart).toFloat();

      J1EncoderMultiplier = inData.substring(J1EncoderMultiplierStart + 1, J2EncoderMultiplierStart).toFloat();
      J2EncoderMultiplier = inData.substring(J2EncoderMultiplierStart + 1, J3EncoderMultiplierStart).toFloat();
      J3EncoderMultiplier = inData.substring(J3EncoderMultiplierStart + 1, J4EncoderMultiplierStart).toFloat();
      J4EncoderMultiplier = inData.substring(J4EncoderMultiplierStart + 1, J5EncoderMultiplierStart).toFloat();
      J5EncoderMultiplier = inData.substring(J5EncoderMultiplierStart + 1, J6EncoderMultiplierStart).toFloat();
      J6EncoderMultiplier = inData.substring(J6EncoderMultiplierStart + 1, J1tDHparStart).toFloat();

      DHparams[0][0] = inData.substring(J1tDHparStart + 1, J2tDHparStart).toFloat();
      DHparams[1][0] = inData.substring(J2tDHparStart + 1, J3tDHparStart).toFloat();
      DHparams[2][0] = inData.substring(J3tDHparStart + 1, J4tDHparStart).toFloat();
      DHparams[3][0] = inData.substring(J4tDHparStart + 1, J5tDHparStart).toFloat();
      DHparams[4][0] = inData.substring(J5tDHparStart + 1, J6tDHparStart).toFloat();
      DHparams[5][0] = inData.substring(J6tDHparStart + 1, J1uDHparStart).toFloat();

      DHparams[0][1] = inData.substring(J1uDHparStart + 1, J2uDHparStart).toFloat();
      DHparams[1][1] = inData.substring(J2uDHparStart + 1, J3uDHparStart).toFloat();
      DHparams[2][1] = inData.substring(J3uDHparStart + 1, J4uDHparStart).toFloat();
      DHparams[3][1] = inData.substring(J4uDHparStart + 1, J5uDHparStart).toFloat();
      DHparams[4][1] = inData.substring(J5uDHparStart + 1, J6uDHparStart).toFloat();
      DHparams[5][1] = inData.substring(J6uDHparStart + 1, J1dDHparStart).toFloat();

      DHparams[0][2] = inData.substring(J1dDHparStart + 1, J2dDHparStart).toFloat();
      DHparams[1][2] = inData.substring(J2dDHparStart + 1, J3dDHparStart).toFloat();
      DHparams[2][2] = inData.substring(J3dDHparStart + 1, J4dDHparStart).toFloat();
      DHparams[3][2] = inData.substring(J4dDHparStart + 1, J5dDHparStart).toFloat();
      DHparams[4][2] = inData.substring(J5dDHparStart + 1, J6dDHparStart).toFloat();
      DHparams[5][2] = inData.substring(J6dDHparStart + 1, J1aDHparStart).toFloat();

      DHparams[0][3] = inData.substring(J1aDHparStart + 1, J2aDHparStart).toFloat();
      DHparams[1][3] = inData.substring(J2aDHparStart + 1, J3aDHparStart).toFloat();
      DHparams[2][3] = inData.substring(J3aDHparStart + 1, J4aDHparStart).toFloat();
      DHparams[3][3] = inData.substring(J4aDHparStart + 1, J5aDHparStart).toFloat();
      DHparams[4][3] = inData.substring(J5aDHparStart + 1, J6aDHparStart).toFloat();
      DHparams[5][3] = inData.substring(J6aDHparStart + 1).toFloat();


      //define total axis travel
      J1AxisDegreeRange = J1axisLimPos + J1axisLimNeg;
      J2axisLim = J2axisLimPos + J2axisLimNeg;
      J3axisLim = J3axisLimPos + J3axisLimNeg;
      J4axisLim = J4axisLimPos + J4axisLimNeg;
      J5axisLim = J5axisLimPos + J5axisLimNeg;
      J6axisLim = J6axisLimPos + J6axisLimNeg;

      //steps full movement of each axis
      J1StepRange = J1AxisDegreeRange * J1StepsPerDegree;
      J2StepRange = J2axisLim * J2StepDeg;
      J3StepRange = J3axisLim * J3StepDeg;
      J4StepRange = J4axisLim * J4StepDeg;
      J5StepRange = J5axisLim * J5StepDeg;
      J6StepRange = J6axisLim * J6StepDeg;

      //step and axis zero
      J1zeroStep = J1axisLimNeg * J1StepsPerDegree;
      J2zeroStep = J2axisLimNeg * J2StepDeg;
      J3zeroStep = J3axisLimNeg * J3StepDeg;
      J4zeroStep = J4axisLimNeg * J4StepDeg;
      J5zeroStep = J5axisLimNeg * J5StepDeg;
      J6zeroStep = J6axisLimNeg * J6StepDeg;

      Serial.print("Done");
    }

    //-----COMMAND CALIBRATE EXTERNAL AXIS---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "CE") {
      // CALIBRATE EXTERNAL AXIS - Configure J7, J8, J9 axis parameters
      // Sets length, rotation range, and step resolution for linear/rotary external axes
      // Parameters: A<length>B<range>C<steps> for each axis
      int J7lengthStart = inData.indexOf('A');
      int J7rotStart = inData.indexOf('B');
      int J7stepsStart = inData.indexOf('C');
      int J8lengthStart = inData.indexOf('D');
      int J8rotStart = inData.indexOf('E');
      int J8stepsStart = inData.indexOf('F');
      int J9lengthStart = inData.indexOf('G');
      int J9rotStart = inData.indexOf('H');
      int J9stepsStart = inData.indexOf('I');

      J7length = inData.substring(J7lengthStart + 1, J7rotStart).toFloat();
      J7rot = inData.substring(J7rotStart + 1, J7stepsStart).toFloat();
      J7steps = inData.substring(J7stepsStart + 1, J8lengthStart).toFloat();

      J8length = inData.substring(J8lengthStart + 1, J8rotStart).toFloat();
      J8rot = inData.substring(J8rotStart + 1, J8stepsStart).toFloat();
      J8steps = inData.substring(J8stepsStart + 1, J9lengthStart).toFloat();

      J9length = inData.substring(J9lengthStart + 1, J9rotStart).toFloat();
      J9rot = inData.substring(J9rotStart + 1, J9stepsStart).toFloat();
      J9steps = inData.substring(J9stepsStart + 1).toFloat();

      J7axisLimNeg = 0;
      J7axisLimPos = J7length;
      J7axisLim = J7axisLimPos + J7axisLimNeg;
      J7StepDeg = J7steps / J7rot;
      J7StepRange = J7axisLim * J7StepDeg;

      J8axisLimNeg = 0;
      J8axisLimPos = J8length;
      J8axisLim = J8axisLimPos + J8axisLimNeg;
      J8StepDeg = J8steps / J8rot;
      J8StepRange = J8axisLim * J8StepDeg;

      J9axisLimNeg = 0;
      J9axisLimPos = J9length;
      J9axisLim = J9axisLimPos + J9axisLimNeg;
      J9StepDeg = J9steps / J9rot;
      J9StepRange = J9axisLim * J9StepDeg;

      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND ZERO J7---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "Z7") {
      J7MasterStep = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "Z8") {
      J8MasterStep = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "Z9") {
      J9MasterStep = 0;
      sendRobotPos();
    }


    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "WT") {
      int WTstart = inData.indexOf('S');
      float WaitTime = inData.substring(WTstart + 1).toFloat();
      int WaitTimeMS = WaitTime * 1000;
      delay(WaitTimeMS);
      Serial.println("WTdone");
    }


    //-----COMMAND SET OUTPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "ON") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, HIGH);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND SET OUTPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "OF") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, LOW);
      delay(5);
      Serial.println("Done");
    }

    //-----COMMAND TO WAIT MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "WJ") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
      int timeout = inData.substring(timoutIndex + 1).toInt();
      unsigned long timeoutMillis = timeout * 1000;
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 1);
        delay(100);
      }
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND TO WAIT MODBUS INPUT---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "WK") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
      int timeout = inData.substring(timoutIndex + 1).toInt();
      unsigned long timeoutMillis = timeout * 1000;
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 2);
        delay(100);
      }
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND TO SET MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "SC") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1).toInt();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 15);
      delay(5);
      Serial.println(result);
    }

    //-----COMMAND TO SET MODBUS OUTPUT REGISTER---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "SO") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1).toInt();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 6);
      delay(5);
      Serial.println(result);
    }


    //-----COMMAND SEND POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "SP") {
      int J1angStart = inData.indexOf('A');
      int J2angStart = inData.indexOf('B');
      int J3angStart = inData.indexOf('C');
      int J4angStart = inData.indexOf('D');
      int J5angStart = inData.indexOf('E');
      int J6angStart = inData.indexOf('F');
      int J7angStart = inData.indexOf('G');
      int J8angStart = inData.indexOf('H');
      int J9angStart = inData.indexOf('I');
      J1MasterStep = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepsPerDegree;
      J2MasterStep = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
      J3MasterStep = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
      J4MasterStep = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
      J5MasterStep = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
      J6MasterStep = ((inData.substring(J6angStart + 1, J7angStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
      J7MasterStep = ((inData.substring(J7angStart + 1, J8angStart).toFloat()) + J7axisLimNeg) * J7StepDeg;
      J8MasterStep = ((inData.substring(J8angStart + 1, J9angStart).toFloat()) + J8axisLimNeg) * J8StepDeg;
      J9MasterStep = ((inData.substring(J9angStart + 1).toFloat()) + J9axisLimNeg) * J9StepDeg;
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "TM") {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int WristConStart = inData.indexOf('W');
      CurrentJointAngle[0] = inData.substring(J1start + 1, J2start).toFloat();
      CurrentJointAngle[1] = inData.substring(J2start + 1, J3start).toFloat();
      CurrentJointAngle[2] = inData.substring(J3start + 1, J4start).toFloat();
      CurrentJointAngle[3] = inData.substring(J4start + 1, J5start).toFloat();
      CurrentJointAngle[4] = inData.substring(J5start + 1, J6start).toFloat();
      CurrentJointAngle[5] = inData.substring(J6start + 1, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1);
      WristCon.trim();

      SolveInverseKinematics();

      String echo = "";
      delay(5);
      Serial.println(inData);
    }


    //-----COMMAND TO CALIBRATE---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "LL") {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int J7start = inData.indexOf('G');
      int J8start = inData.indexOf('H');
      int J9start = inData.indexOf('I');

      int J1calstart = inData.indexOf('J');
      int J2calstart = inData.indexOf('K');
      int J3calstart = inData.indexOf('L');
      int J4calstart = inData.indexOf('M');
      int J5calstart = inData.indexOf('N');
      int J6calstart = inData.indexOf('O');
      int J7calstart = inData.indexOf('P');
      int J8calstart = inData.indexOf('Q');
      int J9calstart = inData.indexOf('R');

      ///
      int J1req = inData.substring(J1start + 1, J2start).toInt();
      int J2req = inData.substring(J2start + 1, J3start).toInt();
      int J3req = inData.substring(J3start + 1, J4start).toInt();
      int J4req = inData.substring(J4start + 1, J5start).toInt();
      int J5req = inData.substring(J5start + 1, J6start).toInt();
      int J6req = inData.substring(J6start + 1, J7start).toInt();
      int J7req = inData.substring(J7start + 1, J8start).toInt();
      int J8req = inData.substring(J8start + 1, J9start).toInt();
      int J9req = inData.substring(J9start + 1, J1calstart).toInt();



      float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
      float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
      float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
      float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
      float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
      float J6calOff = inData.substring(J6calstart + 1, J7calstart).toFloat();
      float J7calOff = inData.substring(J7calstart + 1, J8calstart).toFloat();
      float J8calOff = inData.substring(J8calstart + 1, J9calstart).toFloat();
      float J9calOff = inData.substring(J9calstart + 1).toFloat();
      ///
      float SpeedIn;
      ///
      int J1Step = 0;
      int J2Step = 0;
      int J3Step = 0;
      int J4Step = 0;
      int J5Step = 0;
      int J6Step = 0;
      int J7Step = 0;
      int J8Step = 0;
      int J9Step = 0;
      ///
      int J1stepCen = 0;
      int J2stepCen = 0;
      int J3stepCen = 0;
      int J4stepCen = 0;
      int J5stepCen = 0;
      int J5step45 = 0;
      int J6stepCen = 0;
      int J7stepCen = 0;
      int J8stepCen = 0;
      int J9stepCen = 0;
      ///
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int Jreq[9] = { J1req, J2req, J3req, J4req, J5req, J6req, J7req, J8req, J9req };
      int JStepLim[9] = { J1StepRange, J2StepRange, J3StepRange, J4StepRange, J5StepRange, J6StepRange, J7StepRange, J8StepRange, J9StepRange };
      int JcalPin[9] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
      int JStep[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

      for (int i = 0; i < 9; i++) {
        if (Jreq[i] == 1) {
          JStep[i] = JStepLim[i];
        }
      }

      //DRIVE TO LIMITS FAST
      SpeedIn = 40;
      driveLimit(JStep, SpeedIn);

      //Backoff
      backOff(J1req, J2req, J3req, J4req, J5req, J6req, J7req, J8req, J9req);

      //DRIVE TO LIMITS SLOW
      SpeedIn = 5;
      driveLimit(JStep, SpeedIn);




      //set master steps and center step

      if (J1req == 1) {
        if (J1CalDir == 1) {
          J1MasterStep = ((J1AxisDegreeRange) + J1calBaseOff + J1calOff) * J1StepsPerDegree;
          J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepsPerDegree;
        } else {
          J1MasterStep = (0 + J1calBaseOff + J1calOff) * J1StepsPerDegree;
          J1stepCen = ((J1axisLimNeg)-J1calBaseOff - J1calOff) * J1StepsPerDegree;
        }
      }
      if (J2req == 1) {
        if (J2CalDir == 1) {
          J2MasterStep = ((J2axisLim) + J2calBaseOff + J2calOff) * J2StepDeg;
          J2stepCen = ((J2axisLimPos) + J2calBaseOff + J2calOff) * J2StepDeg;
        } else {
          J2MasterStep = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
          J2stepCen = ((J2axisLimNeg)-J2calBaseOff - J2calOff) * J2StepDeg;
        }
      }
      if (J3req == 1) {
        if (J3CalDir == 1) {
          J3MasterStep = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
          J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
        } else {
          J3MasterStep = (0 + J3calBaseOff + J3calOff) * J3StepDeg;
          J3stepCen = ((J3axisLimNeg)-J3calBaseOff - J3calOff) * J3StepDeg;
        }
      }
      if (J4req == 1) {
        if (J4CalDir == 1) {
          J4MasterStep = ((J4axisLim) + J4calBaseOff + J4calOff) * J4StepDeg;
          J4stepCen = ((J4axisLimPos) + J4calBaseOff + J4calOff) * J4StepDeg;
        } else {
          J4MasterStep = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
          J4stepCen = ((J4axisLimNeg)-J4calBaseOff - J4calOff) * J4StepDeg;
        }
      }
      if (J5req == 1) {
        if (J5CalDir == 1) {
          J5MasterStep = ((J5axisLim) + J5calBaseOff + J5calOff) * J5StepDeg;
          J5stepCen = ((J5axisLimPos) + J5calBaseOff + J5calOff) * J5StepDeg;
          J5step45 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) - 45) * J5StepDeg;
        } else {
          J5MasterStep = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
          J5stepCen = ((J5axisLimNeg)-J5calBaseOff - J5calOff) * J5StepDeg;
          J5step45 = (((J5axisLimNeg)-J5calBaseOff - J5calOff) + 45) * J5StepDeg;
        }
      }
      if (J6req == 1) {
        if (J6CalDir == 1) {
          J6MasterStep = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
          J6stepCen = ((J6axisLimPos) + J6calBaseOff + J6calOff) * J6StepDeg;
        } else {
          J6MasterStep = (0 + J6calBaseOff + J6calOff) * J6StepDeg;
          J6stepCen = ((J6axisLimNeg)-J6calBaseOff - J6calOff) * J6StepDeg;
        }
      }
      if (J7req == 1) {
        if (J7CalDir == 1) {
          J7MasterStep = ((J7axisLim) + J7calBaseOff + J7calOff) * J7StepDeg;
          J7stepCen = ((J7axisLimPos) + J7calBaseOff + J7calOff) * J7StepDeg;
        } else {
          J7MasterStep = (0 + J7calBaseOff + J7calOff) * J7StepDeg;
          J7stepCen = ((J7axisLimNeg)-J7calBaseOff - J7calOff) * J7StepDeg;
        }
      }
      if (J8req == 1) {
        if (J8CalDir == 1) {
          J8MasterStep = ((J8axisLim) + J8calBaseOff + J8calOff) * J8StepDeg;
          J8stepCen = ((J8axisLimPos) + J8calBaseOff + J8calOff) * J8StepDeg;
        } else {
          J8MasterStep = (0 + J8calBaseOff + J8calOff) * J8StepDeg;
          J8stepCen = ((J8axisLimNeg)-J8calBaseOff - J8calOff) * J8StepDeg;
        }
      }
      if (J9req == 1) {
        if (J9CalDir == 1) {
          J9MasterStep = ((J9axisLim) + J9calBaseOff + J9calOff) * J9StepDeg;
          J9stepCen = ((J9axisLimPos) + J9calBaseOff + J9calOff) * J9StepDeg;
        } else {
          J9MasterStep = (0 + J9calBaseOff + J9calOff) * J9StepDeg;
          J9stepCen = ((J9axisLimNeg)-J9calBaseOff - J9calOff) * J9StepDeg;
        }
      }


      //move to center
      /// J1 ///
      if (J1CalDir) {
        J1dir = 0;
      } else {
        J1dir = 1;
      }
      /// J2 ///
      if (J2CalDir) {
        J2dir = 0;
      } else {
        J2dir = 1;
      }
      /// J3 ///
      if (J3CalDir) {
        J3dir = 0;
      } else {
        J3dir = 1;
      }
      /// J4 ///
      if (J4CalDir) {
        J4dir = 0;
      } else {
        J4dir = 1;
      }
      /// J5 ///
      if (J5CalDir) {
        J5dir = 0;
      } else {
        J5dir = 1;
      }
      /// J6 ///
      if (J6CalDir) {
        J6dir = 0;
      } else {
        J6dir = 1;
      }
      /// J7 ///
      if (J7CalDir) {
        J7dir = 0;
      } else {
        J7dir = 1;
      }
      /// J8 ///
      if (J8CalDir) {
        J8dir = 0;
      } else {
        J8dir = 1;
      }
      /// J9 ///
      if (J9CalDir) {
        J9dir = 0;
      } else {
        J9dir = 1;
      }

      float ACCspd = 10;
      float DCCspd = 10;
      String SpeedType = "p";
      float SpeedVal = 50;
      float ACCramp = 50;
      //move to center positions except J5 to 45deg
      driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5step45, J6stepCen, J7stepCen, J8stepCen, J9stepCen, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      sendRobotPos();
      inData = "";  // Clear recieved buffer
    }


    //----- LIVE CARTESIAN JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "LC") {
      delay(5);
      Serial.println();


      updatePos();

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";


      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 0;
      float DCCspd = 0;
      float ACCramp = 10;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = "";  // Clear recieved buffer

      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];


      while (JogInPoc == true) {

        if (Vector == 10) {
          xyzuvw_In[0] = xyzuvw_Out[0] - JogStepInc;
        }
        if (Vector == 11) {
          xyzuvw_In[0] = xyzuvw_Out[0] + JogStepInc;
        }

        if (Vector == 20) {
          xyzuvw_In[1] = xyzuvw_Out[1] - JogStepInc;
        }
        if (Vector == 21) {
          xyzuvw_In[1] = xyzuvw_Out[1] + JogStepInc;
        }

        if (Vector == 30) {
          xyzuvw_In[2] = xyzuvw_Out[2] - JogStepInc;
        }
        if (Vector == 31) {
          xyzuvw_In[2] = xyzuvw_Out[2] + JogStepInc;
        }

        if (Vector == 40) {
          xyzuvw_In[3] = xyzuvw_Out[3] - JogStepInc;
        }
        if (Vector == 41) {
          xyzuvw_In[3] = xyzuvw_Out[3] + JogStepInc;
        }

        if (Vector == 50) {
          xyzuvw_In[4] = xyzuvw_Out[4] - JogStepInc;
        }
        if (Vector == 51) {
          xyzuvw_In[4] = xyzuvw_Out[4] + JogStepInc;
        }

        if (Vector == 60) {
          xyzuvw_In[5] = xyzuvw_Out[5] - JogStepInc;
        }
        if (Vector == 61) {
          xyzuvw_In[5] = xyzuvw_Out[5] + JogStepInc;
        }

        SolveInverseKinematics();

        //calc destination motor steps
        int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;
        int J7StepDelta = 0;
        int J8StepDelta = 0;
        int J9StepDelta = 0;

        //determine motor directions
        J1dir = (J1StepDelta <= 0) ? 1 : 0;
        J2dir = (J2StepDelta <= 0) ? 1 : 0;
        J3dir = (J3StepDelta <= 0) ? 1 : 0;
        J4dir = (J4StepDelta <= 0) ? 1 : 0;
        J5dir = (J5StepDelta <= 0) ? 1 : 0;
        J6dir = (J6StepDelta <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "LJ") {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";


      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 0;
      float DCCspd = 0;
      float ACCramp = 10;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = "";  // Clear recieved buffer

      //clear serial
      delay(5);
      Serial.println();
      updatePos();

      float J1Angle = CurrentJointAngle[0];
      float J2Angle = CurrentJointAngle[1];
      float J3Angle = CurrentJointAngle[2];
      float J4Angle = CurrentJointAngle[3];
      float J5Angle = CurrentJointAngle[4];
      float J6Angle = CurrentJointAngle[5];
      float J7Angle = J7_pos;
      float J8Angle = J8_pos;
      float J9Angle = J9_pos;
      float xyzuvw_In[6];

      while (JogInPoc == true) {

        if (Vector == 10) {
          J1Angle = CurrentJointAngle[0] - .25;
        }
        if (Vector == 11) {
          J1Angle = CurrentJointAngle[0] + .25;
        }

        if (Vector == 20) {
          J2Angle = CurrentJointAngle[1] - .25;
        }
        if (Vector == 21) {
          J2Angle = CurrentJointAngle[1] + .25;
        }

        if (Vector == 30) {
          J3Angle = CurrentJointAngle[2] - .25;
        }
        if (Vector == 31) {
          J3Angle = CurrentJointAngle[2] + .25;
        }

        if (Vector == 40) {
          J4Angle = CurrentJointAngle[3] - .25;
        }
        if (Vector == 41) {
          J4Angle = CurrentJointAngle[3] + .25;
        }

        if (Vector == 50) {
          J5Angle = CurrentJointAngle[4] - .25;
        }
        if (Vector == 51) {
          J5Angle = CurrentJointAngle[4] + .25;
        }

        if (Vector == 60) {
          J6Angle = CurrentJointAngle[5] - .25;
        }
        if (Vector == 61) {
          J6Angle = CurrentJointAngle[5] + .25;
        }
        if (Vector == 70) {
          J7Angle = J7_pos - .25;
        }
        if (Vector == 71) {
          J7Angle = J7_pos + .25;
        }
        if (Vector == 80) {
          J8Angle = J8_pos - .25;
        }
        if (Vector == 81) {
          J8Angle = J8_pos + .25;
        }
        if (Vector == 90) {
          J9Angle = J9_pos - .25;
        }
        if (Vector == 91) {
          J9Angle = J9_pos + .25;
        }

        //calc destination motor steps
        int J1FutureMasterStep = (J1Angle + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (J2Angle + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (J3Angle + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (J4Angle + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (J5Angle + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (J6Angle + J6axisLimNeg) * J6StepDeg;
        int J7futStepM = (J7Angle + J7axisLimNeg) * J7StepDeg;
        int J8futStepM = (J8Angle + J8axisLimNeg) * J8StepDeg;
        int J9futStepM = (J9Angle + J9axisLimNeg) * J9StepDeg;

        //calc delta from current to destination
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;
        int J7StepDelta = J7MasterStep - J7futStepM;
        int J8StepDelta = J8MasterStep - J8futStepM;
        int J9StepDelta = J9MasterStep - J9futStepM;

        //determine motor directions
        J1dir = (J1StepDelta <= 0) ? 1 : 0;
        J2dir = (J2StepDelta <= 0) ? 1 : 0;
        J3dir = (J3StepDelta <= 0) ? 1 : 0;
        J4dir = (J4StepDelta <= 0) ? 1 : 0;
        J5dir = (J5StepDelta <= 0) ? 1 : 0;
        J6dir = (J6StepDelta <= 0) ? 1 : 0;
        J7dir = (J7StepDelta <= 0) ? 1 : 0;
        J8dir = (J8StepDelta <= 0) ? 1 : 0;
        J9dir = (J9StepDelta <= 0) ? 1 : 0;

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
          J6axisFault = 1;
        }
        if ((J7dir == 1 and (J7MasterStep + J7StepDelta > J7StepRange)) or (J7dir == 0 and (J7MasterStep - J7StepDelta < 0))) {
          J7axisFault = 1;
        }
        if ((J8dir == 1 and (J8MasterStep + J8StepDelta > J8StepRange)) or (J8dir == 0 and (J8MasterStep - J8StepDelta < 0))) {
          J8axisFault = 1;
        }
        if ((J9dir == 1 and (J9MasterStep + J9StepDelta > J9StepRange)) or (J9dir == 0 and (J9MasterStep - J9StepDelta < 0))) {
          J9axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- LIVE TOOL JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "LT") {
      delay(5);
      Serial.println();

      updatePos();

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TRaxisFault = 0;
      int TotalAxisFault = 0;

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[3];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[5];

      bool JogInPoc = true;
      Alarm = "0";

      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 100;
      float DCCspd = 100;
      float ACCramp = 100;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = "";  // Clear recieved buffer


      Xtool = Robot_Kin_Tool[0];
      Ytool = Robot_Kin_Tool[1];
      Ztool = Robot_Kin_Tool[2];
      RXtool = Robot_Kin_Tool[3];
      RYtool = Robot_Kin_Tool[4];
      RZtool = Robot_Kin_Tool[5];

      CurrentJointAngle[0] = (J1MasterStep - J1zeroStep) / J1StepsPerDegree;
      CurrentJointAngle[1] = (J2MasterStep - J2zeroStep) / J2StepDeg;
      CurrentJointAngle[2] = (J3MasterStep - J3zeroStep) / J3StepDeg;
      CurrentJointAngle[3] = (J4MasterStep - J4zeroStep) / J4StepDeg;
      CurrentJointAngle[4] = (J5MasterStep - J5zeroStep) / J5StepDeg;
      CurrentJointAngle[5] = (J6MasterStep - J6zeroStep) / J6StepDeg;

      while (JogInPoc == true) {

        Xtool = Robot_Kin_Tool[0];
        Ytool = Robot_Kin_Tool[1];
        Ztool = Robot_Kin_Tool[2];
        RXtool = Robot_Kin_Tool[3];
        RYtool = Robot_Kin_Tool[4];
        RZtool = Robot_Kin_Tool[5];

        if (Vector == 10) {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + .25;
        }
        if (Vector == 11) {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - .25;
        }

        if (Vector == 20) {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + .25;
        }
        if (Vector == 21) {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - .25;
        }

        if (Vector == 30) {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + .25;
        }
        if (Vector == 31) {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - .25;
        }

        if (Vector == 60) {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + .25 * M_PI / 180;
        }
        if (Vector == 61) {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - .25 * M_PI / 180;
        }

        if (Vector == 50) {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + .25 * M_PI / 180;
        }
        if (Vector == 51) {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - .25 * M_PI / 180;
        }

        if (Vector == 40) {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + .25 * M_PI / 180;
        }
        if (Vector == 41) {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - .25 * M_PI / 180;
        }



        xyzuvw_In[0] = xyzuvw_Out[0];
        xyzuvw_In[1] = xyzuvw_Out[1];
        xyzuvw_In[2] = xyzuvw_Out[2];
        xyzuvw_In[3] = xyzuvw_Out[3];
        xyzuvw_In[4] = xyzuvw_Out[4];
        xyzuvw_In[5] = xyzuvw_Out[5];

        SolveInverseKinematics();

        Robot_Kin_Tool[0] = Xtool;
        Robot_Kin_Tool[1] = Ytool;
        Robot_Kin_Tool[2] = Ztool;
        Robot_Kin_Tool[3] = RXtool;
        Robot_Kin_Tool[4] = RYtool;
        Robot_Kin_Tool[5] = RZtool;

        //calc destination motor steps
        int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;
        int J7StepDelta = 0;
        int J8StepDelta = 0;
        int J9StepDelta = 0;

        //determine motor directions
        J1dir = (J1StepDelta <= 0) ? 1 : 0;
        J2dir = (J2StepDelta <= 0) ? 1 : 0;
        J3dir = (J3StepDelta <= 0) ? 1 : 0;
        J4dir = (J4StepDelta <= 0) ? 1 : 0;
        J5dir = (J5StepDelta <= 0) ? 1 : 0;
        J6dir = (J6StepDelta <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- Jog T ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "JT") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[3];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[5];

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      String Alarm = "0";

      int SPstart = inData.indexOf('S');
      int AcStart = inData.indexOf('G');
      int DcStart = inData.indexOf('H');
      int RmStart = inData.indexOf('I');
      int LoopModeStart = inData.indexOf("Lm");

      String Dir = inData.substring(0, 2);  // this should be Z0 or Z1
      float Dist = inData.substring(2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 1, DcStart).toInt();
      float DCCspd = inData.substring(DcStart + 1, RmStart).toInt();
      float ACCramp = inData.substring(RmStart + 1, LoopModeStart).toInt();
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      if (Dir == "X0") {
        Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - Dist;
      } else if (Dir == "X1") {
        Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + Dist;
      } else if (Dir == "Y0") {
        Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - Dist;
      } else if (Dir == "Y1") {
        Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + Dist;
      } else if (Dir == "Z0") {
        Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - Dist;
      } else if (Dir == "Z1") {
        Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + Dist;
      } else if (Dir == "R0") {
        Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - Dist * M_PI / 180;
      } else if (Dir == "R1") {
        Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + Dist * M_PI / 180;
      } else if (Dir == "P0") {
        Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - Dist * M_PI / 180;
      } else if (Dir == "P1") {
        Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + Dist * M_PI / 180;
      } else if (Dir == "W0") {
        Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - Dist * M_PI / 180;
      } else if (Dir == "W1") {
        Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + Dist * M_PI / 180;
      }


      CurrentJointAngle[0] = (J1MasterStep - J1zeroStep) / J1StepsPerDegree;
      CurrentJointAngle[1] = (J2MasterStep - J2zeroStep) / J2StepDeg;
      CurrentJointAngle[2] = (J3MasterStep - J3zeroStep) / J3StepDeg;
      CurrentJointAngle[3] = (J4MasterStep - J4zeroStep) / J4StepDeg;
      CurrentJointAngle[4] = (J5MasterStep - J5zeroStep) / J5StepDeg;
      CurrentJointAngle[5] = (J6MasterStep - J6zeroStep) / J6StepDeg;


      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];

      SolveInverseKinematics();

      Robot_Kin_Tool[0] = Xtool;
      Robot_Kin_Tool[1] = Ytool;
      Robot_Kin_Tool[2] = Ztool;
      Robot_Kin_Tool[3] = RZtool;
      Robot_Kin_Tool[4] = RYtool;
      Robot_Kin_Tool[5] = RXtool;


      //calc destination motor steps
      int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
      int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
      int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
      int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
      int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
      int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

      //calc delta from current to destination
      int J1StepDelta = J1MasterStep - J1FutureMasterStep;
      int J2StepDelta = J2MasterStep - J2FutureMasterStep;
      int J3StepDelta = J3MasterStep - J3FutureMasterStep;
      int J4StepDelta = J4MasterStep - J4FutureMasterStep;
      int J5StepDelta = J5MasterStep - J5FutureMasterStep;
      int J6StepDelta = J6MasterStep - J6FutureMasterStep;
      int J7StepDelta = 0;
      int J8StepDelta = 0;
      int J9StepDelta = 0;

      //determine motor directions
      J1dir = (J1StepDelta <= 0) ? 1 : 0;
      J2dir = (J2StepDelta <= 0) ? 1 : 0;
      J3dir = (J3StepDelta <= 0) ? 1 : 0;
      J4dir = (J4StepDelta <= 0) ? 1 : 0;
      J5dir = (J5StepDelta <= 0) ? 1 : 0;
      J6dir = (J6StepDelta <= 0) ? 1 : 0;
      J7dir = 0;
      J8dir = 0;
      J9dir = 0;

      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
        J6axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

      debug = String(SpeedVal);
      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }





    //----- MOVE V ------ VISION OFFSET ----------------------------------
    //-----------------------------------------------------------------------
    else if (function == "MV") {
      // MOVE VISION - Cartesian move with vision-based tool orientation correction
      // Applies vision-detected angle offset to tool frame before motion planning
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int VisRotStart = inData.indexOf("Vr");
      int LoopModeStart = inData.indexOf("Lm");

      xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, VisRotStart);
      float VisRot = inData.substring(VisRotStart + 2, LoopModeStart).toFloat();
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      //get current tool rotation
      float RXtool = Robot_Kin_Tool[5];


      // offset tool rotation by the found vision angle
      Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - VisRot * M_PI / 180;

      //solve kinematics
      SolveInverseKinematics();

      //calc destination motor steps
      int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
      int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
      int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
      int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
      int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
      int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;


      //calc delta from current to destination
      int J1StepDelta = J1MasterStep - J1FutureMasterStep;
      int J2StepDelta = J2MasterStep - J2FutureMasterStep;
      int J3StepDelta = J3MasterStep - J3FutureMasterStep;
      int J4StepDelta = J4MasterStep - J4FutureMasterStep;
      int J5StepDelta = J5MasterStep - J5FutureMasterStep;
      int J6StepDelta = J6MasterStep - J6FutureMasterStep;
      int J7StepDelta = J7MasterStep - J7futStepM;
      int J8StepDelta = J8MasterStep - J8futStepM;
      int J9StepDelta = J9MasterStep - J9futStepM;

      // put tool roation back where it was
      Robot_Kin_Tool[5] = RXtool;

      //determine motor directions
      J1dir = (J1StepDelta <= 0) ? 1 : 0;
      J2dir = (J2StepDelta <= 0) ? 1 : 0;
      J3dir = (J3StepDelta <= 0) ? 1 : 0;
      J4dir = (J4StepDelta <= 0) ? 1 : 0;
      J5dir = (J5StepDelta <= 0) ? 1 : 0;
      J6dir = (J6StepDelta <= 0) ? 1 : 0;
      J7dir = (J7StepDelta <= 0) ? 1 : 0;
      J8dir = (J8StepDelta <= 0) ? 1 : 0;
      J9dir = (J9StepDelta <= 0) ? 1 : 0;



      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7MasterStep + J7StepDelta > J7StepRange)) or (J7dir == 0 and (J7MasterStep - J7StepDelta < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8MasterStep + J8StepDelta > J8StepRange)) or (J8dir == 0 and (J8MasterStep - J8StepDelta < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9MasterStep + J9StepDelta > J9StepRange)) or (J9dir == 0 and (J9MasterStep - J9StepDelta < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
    //-----------------------------------------------------------------------

    else if (function == "RJ") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int J1stepStart = inData.indexOf("A");
      int J2stepStart = inData.indexOf("B");
      int J3stepStart = inData.indexOf("C");
      int J4stepStart = inData.indexOf("D");
      int J5stepStart = inData.indexOf("E");
      int J6stepStart = inData.indexOf("F");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      float J1Angle;
      float J2Angle;
      float J3Angle;
      float J4Angle;
      float J5Angle;
      float J6Angle;
      //Read angles from command
      J1Angle = inData.substring(J1stepStart + 1, J2stepStart).toFloat();
      J2Angle = inData.substring(J2stepStart + 1, J3stepStart).toFloat();
      J3Angle = inData.substring(J3stepStart + 1, J4stepStart).toFloat();
      J4Angle = inData.substring(J4stepStart + 1, J5stepStart).toFloat();
      J5Angle = inData.substring(J5stepStart + 1, J6stepStart).toFloat();
      J6Angle = inData.substring(J6stepStart + 1, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      //read Loop Modes from command
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();
      
      if (J1LoopMode == 0 || J2LoopMode == 0 || J3LoopMode == 0 || J4LoopMode == 0 || J5LoopMode == 0 || J6LoopMode == 0) {
        closedLoopTrue = 1;
      }
      //Calculates steps from angle
      int J1FutureMasterStep = (J1Angle + J1axisLimNeg) * J1StepsPerDegree;
      int J2FutureMasterStep = (J2Angle + J2axisLimNeg) * J2StepDeg;
      int J3FutureMasterStep = (J3Angle + J3axisLimNeg) * J3StepDeg;
      int J4FutureMasterStep = (J4Angle + J4axisLimNeg) * J4StepDeg;
      int J5FutureMasterStep = (J5Angle + J5axisLimNeg) * J5StepDeg;
      int J6FutureMasterStep = (J6Angle + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;
      if (closedLoop){
        readEncoders();
      }
      //calc delta from current to destination
      int J1StepDelta = J1MasterStep - J1FutureMasterStep;
      int J2StepDelta = J2MasterStep - J2FutureMasterStep;
      int J3StepDelta = J3MasterStep - J3FutureMasterStep;
      int J4StepDelta = J4MasterStep - J4FutureMasterStep;
      int J5StepDelta = J5MasterStep - J5FutureMasterStep;
      int J6StepDelta = J6MasterStep - J6FutureMasterStep;
      int J7StepDelta = J7MasterStep - J7futStepM;
      int J8StepDelta = J8MasterStep - J8futStepM;
      int J9StepDelta = J9MasterStep - J9futStepM;


      //determine motor directions
      J1dir = (J1StepDelta <= 0) ? 1 : 0;
      J2dir = (J2StepDelta <= 0) ? 1 : 0;
      J3dir = (J3StepDelta <= 0) ? 1 : 0;
      J4dir = (J4StepDelta <= 0) ? 1 : 0;
      J5dir = (J5StepDelta <= 0) ? 1 : 0;
      J6dir = (J6StepDelta <= 0) ? 1 : 0;
      J7dir = (J7StepDelta <= 0) ? 1 : 0;
      J8dir = (J8StepDelta <= 0) ? 1 : 0;
      J9dir = (J9StepDelta <= 0) ? 1 : 0;


      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7MasterStep + J7StepDelta > J7StepRange)) or (J7dir == 0 and (J7MasterStep - J7StepDelta < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8MasterStep + J8StepDelta > J8StepRange)) or (J8dir == 0 and (J8MasterStep - J8StepDelta < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9MasterStep + J9StepDelta > J9StepRange)) or (J9dir == 0 and (J9MasterStep - J9StepDelta < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        if (closedLoop){
          checkEncoders();
        }
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
      }


      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE L ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "ML" and flag == "") {
      // MOVE LINEAR - Cartesian linear motion with advanced motion profiling
      // Supports spline lookahead, corner rounding, and motion control parameters
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float curDelay;

      String nextCMDtype;
      String test;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";

      float curWayDis;
      float speedSP;

      float Xvect;
      float Yvect;
      float Zvect;
      float RZvect;
      float RYvect;
      float RXvect;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");
      int DisWristStart = inData.indexOf("Q");


      xyzuvw_Temp[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_Temp[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_Temp[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_Temp[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_Temp[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_Temp[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2, DisWristStart);
      String DisWrist = inData.substring(DisWristStart + 1);
      DisWrist.trim();

      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      // Extract loop modes for external axes J7, J8, J9 from loop mode string
      // ===== ROUNDING/LOOKAHEAD CORNER ARC LOGIC ===== 
      // When splineTrue is enabled, prepare next command from buffer to calculate corner rounding arc
      if (cmdBuffer2 != "") {
        // Get next command from buffer and extract command type and parameters
        checkData = cmdBuffer2;
        checkData.trim();
        // Extract command type (first character after space)
        nextCMDtype = checkData.substring(0, 1);
        // Extract command parameters (skip "X" or other prefix)
        checkData = checkData.substring(2);
      }
      // If next command is motion (M*) and rounding enabled, calculate corner rounding arc between current and next move
      if (splineTrue == true and Rounding > 0 and nextCMDtype == "M") {
        // Update current position before calculating rounding arc
        updatePos();
        // Calculate direction vector from current endpoint to target position
        float Xvect = xyzuvw_Temp[0] - xyzuvw_Out[0];
        float Yvect = xyzuvw_Temp[1] - xyzuvw_Out[1];
        float Zvect = xyzuvw_Temp[2] - xyzuvw_Out[2];
        // Calculate rotation vector components for RZ, RY, RX orientations
        float RZvect = xyzuvw_Temp[3] - xyzuvw_Out[3];
        float RYvect = xyzuvw_Temp[4] - xyzuvw_Out[4];
        float RXvect = xyzuvw_Temp[5] - xyzuvw_Out[5];
        // Store current position as start point for rounding arc
        float Xstart = xyzuvw_Out[0];
        float Ystart = xyzuvw_Out[1];
        float Zstart = xyzuvw_Out[2];
        float RZstart = xyzuvw_Out[3];
        float RYstart = xyzuvw_Out[4];
        float RXstart = xyzuvw_Out[5];
        // Calculate total line distance using 6D norm (XYZ + Rotation components)
        float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        // Limit rounding radius to 45% of line distance to prevent overshooting
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        // Calculate percentage distance along line where rounding starts (1 - Rounding/distance)
        float newDistPerc = 1 - (Rounding / lineDist);
        // Calculate cropped destination point (reduced endpoint before rounding arc begins)
        xyzuvw_In[0] = Xstart + (Xvect * newDistPerc);
        xyzuvw_In[1] = Ystart + (Yvect * newDistPerc);
        xyzuvw_In[2] = Zstart + (Zvect * newDistPerc);
        xyzuvw_In[3] = RZstart + (RZvect * newDistPerc);
        xyzuvw_In[4] = RYstart + (RYvect * newDistPerc);
        xyzuvw_In[5] = RXstart + (RXvect * newDistPerc);
        // Parse next command for its target position to calculate rounding arc
        xStart = checkData.indexOf("X");
        yStart = checkData.indexOf("Y");
        zStart = checkData.indexOf("Z");
        rzStart = checkData.indexOf("Rz");
        ryStart = checkData.indexOf("Ry");
        rxStart = checkData.indexOf("Rx");
        J7Start = checkData.indexOf("J7");
        J8Start = checkData.indexOf("J8");
        J9Start = checkData.indexOf("J9");
        // Extract arc endpoint from next command (the far end of the rounding arc)
        rndArcEnd[0] = checkData.substring(xStart + 1, yStart).toFloat();
        rndArcEnd[1] = checkData.substring(yStart + 1, zStart).toFloat();
        rndArcEnd[2] = checkData.substring(zStart + 1, rzStart).toFloat();
        rndArcEnd[3] = checkData.substring(rzStart + 2, ryStart).toFloat();
        rndArcEnd[4] = checkData.substring(ryStart + 2, rxStart).toFloat();
        rndArcEnd[5] = checkData.substring(rxStart + 2, J7Start).toFloat();
        // Calculate vector from current endpoint (xyzuvw_Temp) to the arc endpoint
        Xvect = rndArcEnd[0] - xyzuvw_Temp[0];
        Yvect = rndArcEnd[1] - xyzuvw_Temp[1];
        Zvect = rndArcEnd[2] - xyzuvw_Temp[2];
        // Calculate rotation vector for arc from current to endpoint
        RZvect = rndArcEnd[3] - xyzuvw_Temp[3];
        RYvect = rndArcEnd[4] - xyzuvw_Temp[4];
        RXvect = rndArcEnd[5] - xyzuvw_Temp[5];
        // Store endpoint as start of arc for arc center calculation
        Xstart = xyzuvw_Temp[0];
        Ystart = xyzuvw_Temp[1];
        Zstart = xyzuvw_Temp[2];
        RZstart = xyzuvw_Temp[3];
        RYstart = xyzuvw_Temp[4];
        RXstart = xyzuvw_Temp[5];
        // Calculate distance from current position to next endpoint
        lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        // Limit arc rounding to 45% of the second line segment distance
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        // Calculate arc end point as rounding percentage along second line
        newDistPerc = (Rounding / lineDist);
        // Calculate where arc ends on the path to next command's target
        rndArcEnd[0] = Xstart + (Xvect * newDistPerc);
        rndArcEnd[1] = Ystart + (Yvect * newDistPerc);
        rndArcEnd[2] = Zstart + (Zvect * newDistPerc);
        rndArcEnd[3] = RZstart + (RZvect * newDistPerc);
        rndArcEnd[4] = RYstart + (RYvect * newDistPerc);
        rndArcEnd[5] = RXstart + (RXvect * newDistPerc);
        // Calculate arc center point (midpoint between cropped endpoint and arc end)
        rndCalcCen[0] = (xyzuvw_In[0] + rndArcEnd[0]) / 2;
        rndCalcCen[1] = (xyzuvw_In[1] + rndArcEnd[1]) / 2;
        rndCalcCen[2] = (xyzuvw_In[2] + rndArcEnd[2]) / 2;
        rndCalcCen[3] = (xyzuvw_In[3] + rndArcEnd[3]) / 2;
        rndCalcCen[4] = (xyzuvw_In[4] + rndArcEnd[4]) / 2;
        rndCalcCen[5] = (xyzuvw_In[5] + rndArcEnd[5]) / 2;
        // Calculate midpoint of arc for interpolation during motion (Bezier curve approximation)
        rndArcMid[0] = (xyzuvw_Temp[0] + rndCalcCen[0]) / 2;
        rndArcMid[1] = (xyzuvw_Temp[1] + rndCalcCen[1]) / 2;
        rndArcMid[2] = (xyzuvw_Temp[2] + rndCalcCen[2]) / 2;
        rndArcMid[3] = (xyzuvw_Temp[3] + rndCalcCen[3]) / 2;
        rndArcMid[4] = (xyzuvw_Temp[4] + rndCalcCen[4]) / 2;
        rndArcMid[5] = (xyzuvw_Temp[5] + rndCalcCen[5]) / 2;
        // Construct arc move command with interpolated via point and endpoint
        rndData = "X" + String(rndArcMid[0]) + "Y" + String(rndArcMid[1]) + "Z" + String(rndArcMid[2]) + "Rz" + String(rndArcMid[3]) + "Ry" + String(rndArcMid[4]) + "Rx" + String(rndArcMid[5]) + "Ex" + String(rndArcEnd[0]) + "Ey" + String(rndArcEnd[1]) + "Ez" + String(rndArcEnd[2]) + "Tr" + String(xyzuvw_Temp[6]) + "S" + SpeedType + String(SpeedVal) + "Ac" + String(ACCspd) + "Dc" + String(DCCspd) + "Rm" + String(ACCramp) + "W" + WristCon;
        // Set function to MA (arc motion) to execute rounding arc
        function = "MA";
        // Flag that rounding arc is active (used to carry speed information)
        rndTrue = true;
      } else {
        // No rounding or next command not available, execute straight line motion to target
        updatePos();
        xyzuvw_In[0] = xyzuvw_Temp[0];
        xyzuvw_In[1] = xyzuvw_Temp[1];
        xyzuvw_In[2] = xyzuvw_Temp[2];
        xyzuvw_In[3] = xyzuvw_Temp[3];
        xyzuvw_In[4] = xyzuvw_Temp[4];
        xyzuvw_In[5] = xyzuvw_Temp[5];
      }

      // Calculate main motion vector from current to target position
      Xvect = xyzuvw_In[0] - xyzuvw_Out[0];
      Yvect = xyzuvw_In[1] - xyzuvw_Out[1];
      Zvect = xyzuvw_In[2] - xyzuvw_Out[2];
      // Calculate orientation change vectors (RX, RY, RZ rotations)
      RZvect = xyzuvw_In[3] - xyzuvw_Out[3];
      RYvect = xyzuvw_In[4] - xyzuvw_Out[4];
      RXvect = xyzuvw_In[5] - xyzuvw_Out[5];

      // Store current position as starting point for motion calculation
      float Xstart = xyzuvw_Out[0];
      float Ystart = xyzuvw_Out[1];
      float Zstart = xyzuvw_Out[2];
      float RZstart = xyzuvw_Out[3];
      float RYstart = xyzuvw_Out[4];
      float RXstart = xyzuvw_Out[5];

      // Calculate total distance for this line movement (6D Euclidean norm)
      float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
      // Only proceed if there is actual motion requested
      if (lineDist > 0) {
        // Calculate number of waypoints based on line distance divided by waypoint spacing
        float wayPts = lineDist / linWayDistSP;
        // Calculate percentage increment between waypoints (1 / number of waypoints)
        float wayPerc = 1 / wayPts;

        // Pre-calculate the entire move and all motion parameters (speeds, accelerations, steps)
        // Solve inverse kinematics to find joint angles for target position
        SolveInverseKinematics();
        // Calculate motor steps required to reach target from solved joint angles
        int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

        // Calculate step deltas (difference between current steps and target steps) for each axis
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;

        // FIND HIGHEST STEP COUNT for acceleration profile calculation (bottleneck axis)
        int HighStep = J1StepDelta;
        if (J2StepDelta > HighStep) {
          HighStep = J2StepDelta;
        }
        if (J3StepDelta > HighStep) {
          HighStep = J3StepDelta;
        }
        if (J4StepDelta > HighStep) {
          HighStep = J4StepDelta;
        }
        if (J5StepDelta > HighStep) {
          HighStep = J5StepDelta;
        }
        if (J6StepDelta > HighStep) {
          HighStep = J6StepDelta;
        }

        // PRE-CALCULATE SPEED PROFILES AND ACCELERATION RAMPS
        float calcStepGap;

        // Determine acceleration/normal/deceleration steps based on profile percentages
        float ACCStep = HighStep * (ACCspd / 100);
        float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
        float DCCStep = HighStep * (DCCspd / 100);

        // Set speed target for seconds mode (multiply by 1.2 for timing calibration)
        if (SpeedType == "s") {
          speedSP = (SpeedVal * 1000000) * 1.2;
        } else if ((SpeedType == "m")) {
          // Set speed target for mm/sec mode (convert distance to time, multiply by 1.2)
          speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
        }

        // Calculate step gap (delay between steps) for seconds or mm per sec speed modes
        if (SpeedType == "s" or SpeedType == "m") {
          // Calculate step gap with no acceleration (theoretical constant speed)
          float zeroStepGap = speedSP / HighStep;
          // Calculate step increment during acceleration phase (how much to reduce delay per step)
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          // Calculate total acceleration time (ramp up period)
          float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          // Calculate normal speed phase time (constant velocity period)
          float zeroNORtime = NORStep * zeroStepGap;
          // Calculate step decrement during deceleration phase (how much to increase delay per step)
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          // Calculate total deceleration time (ramp down period)
          float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
          // Calculate total move time across all three phases
          float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
          // Calculate overclock percentage to hit exact speed target (time adjustment factor)
          float overclockPerc = speedSP / zeroTOTtime;
          // Apply overclock percentage to final step gap
          calcStepGap = zeroStepGap * overclockPerc;
          // Enforce minimum speed limit (prevent stepping too fast for microcontroller)
          if (calcStepGap <= minSpeedDelay) {
            calcStepGap = minSpeedDelay;
            // Flag speed violation for user notification
            speedViolation = "1";
          }
        }

        // Calculate step gap for percentage speed mode (percentage of max speed)
        else if (SpeedType == "p") {
          calcStepGap = minSpeedDelay / (SpeedVal / 100);
        }

        // Calculate final step increments for acceleration and deceleration ramps
        float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
        float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
        // Calculate starting delay for acceleration (ensures smooth ramp-up from stop)
        float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
        // Calculate ending delay for deceleration (ensures smooth ramp-down to stop)
        float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;

        // Calculate waypoint acceleration/normal/deceleration counts
        float ACCwayPts = wayPts * (ACCspd / 100);
        float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
        float DCCwayPts = wayPts * (DCCspd / 100);

        // Calculate delay change per waypoint for smooth acceleration ramp
        float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
        float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

        // Set initial delay for first waypoint (start of acceleration)
        if (rndTrue == true) {
          // If rounding arc active, use speed from previous arc move
          curDelay = rndSpeed;
        } else {
          // Otherwise use calculated acceleration start delay
          curDelay = calcACCstartDel;
        }

        // Calculate external axis (J7, J8, J9) step increments across waypoints
        int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
        // Distribute J7 steps evenly across waypoints for smooth motion
        int J7StepDelta = (J7MasterStep - J7futStepM) / (wayPts - 1);
        int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
        // Distribute J8 steps evenly across waypoints
        int J8StepDelta = (J8MasterStep - J8futStepM) / (wayPts - 1);
        int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;
        // Distribute J9 steps evenly across waypoints
        int J9StepDelta = (J9MasterStep - J9futStepM) / (wayPts - 1);

        // Determine J7 motion direction (1=backward, 0=forward)
        if (J7StepDelta <= 0) {
          J7dir = 1;
        } else {
          J7dir = 0;
        }

        // Determine J8 motion direction
        if (J8StepDelta <= 0) {
          J8dir = 1;
        } else {
          J8dir = 0;
        }

        // Determine J9 motion direction
        if (J9StepDelta <= 0) {
          J9dir = 1;
        } else {
          J9dir = 0;
        }

        // Reset encoder fault detection before motion begins
        resetEncoders();
        /////////////////////////////////////////////////
        // Loop through all waypoints to interpolate motion
        for (int i = 0; i <= wayPts + 1; i++) {

          // CALCULATE CURRENT WAYPOINT DELAY WITH ACCELERATION/DECELERATION RAMP
          // During acceleration phase, decrease delay to speed up
          if (i <= ACCwayPts) {
            curDelay = curDelay - (ACCwayInc);
          } else if (i >= (wayPts - DCCwayPts)) {
            // During deceleration phase, increase delay to slow down
            curDelay = curDelay + (DCCwayInc);
          } else {
            // During normal speed phase, maintain constant delay
            curDelay = calcStepGap;
          }

          // Override with constant delay (ensures motion happens at calculated speed)
          curDelay = calcStepGap;

          // Calculate percentage along line (0 to 1) for this waypoint
          float curWayPerc = wayPerc * i;
          // Interpolate X, Y, Z position at this waypoint
          xyzuvw_In[0] = Xstart + (Xvect * curWayPerc);
          xyzuvw_In[1] = Ystart + (Yvect * curWayPerc);
          xyzuvw_In[2] = Zstart + (Zvect * curWayPerc);
          // Interpolate orientation (Rz, Ry, Rx rotation angles)
          xyzuvw_In[3] = RZstart + (RZvect * curWayPerc);
          xyzuvw_In[4] = RYstart + (RYvect * curWayPerc);
          xyzuvw_In[5] = RXstart + (RXvect * curWayPerc);

          // Solve inverse kinematics for this waypoint position
          SolveInverseKinematics();

          // Calculate destination motor steps for this waypoint
          int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
          int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
          int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
          int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
          int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
          int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

          // Calculate step delta from current to target for this waypoint
          int J1StepDelta = J1MasterStep - J1FutureMasterStep;
          int J2StepDelta = J2MasterStep - J2FutureMasterStep;
          int J3StepDelta = J3MasterStep - J3FutureMasterStep;
          int J4StepDelta = J4MasterStep - J4FutureMasterStep;
          int J5StepDelta = J5MasterStep - J5FutureMasterStep;
          int J6StepDelta = J6MasterStep - J6FutureMasterStep;

          // Determine motor rotation directions (1=negative direction, 0=positive)
          J1dir = (J1StepDelta <= 0) ? 1 : 0;
          J2dir = (J2StepDelta <= 0) ? 1 : 0;
          J3dir = (J3StepDelta <= 0) ? 1 : 0;
          J4dir = (J4StepDelta <= 0) ? 1 : 0;
          J5dir = (J5StepDelta <= 0) ? 1 : 0;
          J6dir = (J6StepDelta <= 0) ? 1 : 0;

          // Determine if requested position is within axis limits (collision check)
          if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
            J6axisFault = 1;
          }
          // Check J7, J8, J9 for axis limit violations
          if ((J7dir == 1 and (J7MasterStep + J7StepDelta > J7StepRange)) or (J7dir == 0 and (J7MasterStep - J7StepDelta < 0))) {
            J7axisFault = 1;
          }
          if ((J8dir == 1 and (J8MasterStep + J8StepDelta > J8StepRange)) or (J8dir == 0 and (J8MasterStep - J8StepDelta < 0))) {
            J8axisFault = 1;
          }
          if ((J9dir == 1 and (J9MasterStep + J9StepDelta > J9StepRange)) or (J9dir == 0 and (J9MasterStep - J9StepDelta < 0))) {
            J9axisFault = 1;
          }
          // Sum all axis faults to determine if move is valid
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

          // Execute motion step if no axis limit errors and kinematics valid
          if (TotalAxisFault == 0 && KinematicError == 0) {
            // Call motion routine with absolute step values and directions
            driveMotorsL(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
            // Update current position tracking after motion
            updatePos();
            // Save current delay for rounding arc speed continuity
            rndSpeed = curDelay;
          } else if (KinematicError == 1) {
            // No solution found in inverse kinematics (unreachable target)
            Alarm = "ER";
            if (splineTrue == false) {
              delay(5);
              Serial.println(Alarm);
            }
          } else {
            // Axis limit error - report which axes exceeded limits
            Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
            if (splineTrue == false) {
              delay(5);
              Serial.println(Alarm);
            }
          }
        }
      }

      // Check encoders for collision/slip detection after motion complete
      checkEncoders();
      // If not in spline mode, report final robot position to user
      if (splineTrue == false) {
        sendRobotPos();
      }
      // Clear input buffer after command processed
      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    // ==================================================================================
    // MOTION COMMANDS - Joint and Cartesian space movement
    // ==================================================================================

    else if (function == "MJ") {
      // MOVE JOINT - Move to target position specified in joint angles
      // Uses inverse kinematics if Cartesian target provided
      moveJ(inData, true, false, false);
    }

    else if (function == "MG") {
      // MOVE GCODE - Move based on parsed G-code command
      // Interprets G-code motion parameters and converts to robot motion
      moveJ(inData, true, false, true);
    }


    // ==================================================================================
    // SD CARD FILE OPERATIONS - Store and retrieve motion programs
    // ==================================================================================

    else if (function == "DG") {
      // DELETE PROGRAM - Remove G-code/motion file from SD card
      // Parameter: Fn<filename>
      // Initialize SD card interface
      SD.begin(BUILTIN_SDCARD);
      // Find position of Fn parameter in command string
      int fileStart = inData.indexOf("Fn");
      // Extract filename after Fn prefix
      String filename = inData.substring(fileStart + 2);
      // Convert String to C-string for SD library compatibility
      const char *fn = filename.c_str();
      // Check if file exists on SD card before attempting deletion
      if (SD.exists(fn)) {
        // Call delete function to remove file from SD card
        deleteSD(filename);
        // Send success response ("P" = Program deleted)
        Serial.println("P");
      } else {
        // Send failure response ("F" = File not found)
        Serial.println("F");
      }
    }

    // READ FILES FROM SD CARD - List all files on SD card directory
    else if (function == "RG") {
      // Declare file object for SD card root directory
      File root;
      // Initialize SD card interface
      SD.begin(BUILTIN_SDCARD);
      // Open root directory "/" to list all files
      root = SD.open("/");
      // Call recursive directory printer to display all files and subdirectories
      printDirectory(root, 0);
    }

    // WRITE COMMAND TO SD CARD - Store motion/G-code command as text file
    else if (function == "WC") {
      // Initialize SD card interface
      SD.begin(BUILTIN_SDCARD);
      // Find position of Fn parameter (filename) in command
      int fileStart = inData.indexOf("Fn");
      // Extract filename string from command
      String filename = inData.substring(fileStart + 2);
      // Convert String to C-string for SD library
      const char *fn = filename.c_str();
      // Extract command data before filename (everything before Fn parameter)
      String info = inData.substring(0, fileStart);
      // Write command data to SD card file
      writeSD(fn, info);
      // Send current robot position back to user after write
      sendRobotPos();
    }

    // PLAY FILE ON SD CARD - Execute pre-stored motion commands from file
    else if (function == "PG") {
      // Declare file object for reading commands
      File gcFile;
      // Declare string to hold each command line from file
      String Cmd;
      // Initialize SD card interface
      SD.begin(BUILTIN_SDCARD);
      // Find position of filename parameter in command
      int fileStart = inData.indexOf("Fn");
      // Extract filename string
      String filename = inData.substring(fileStart + 2);
      // Convert String to C-string for SD library
      const char *fn = filename.c_str();
      // Attempt to open file from SD card
      gcFile = SD.open(fn);
      // Check if file open was unsuccessful (null file pointer)
      if (!gcFile) {
        // Send error response (EG = G-code Error)
        Serial.println("EG");
        // Infinite loop to halt execution on file error
        while (1)
          ;
      }
      // Loop while file has data AND emergency stop not triggered
      while (gcFile.available() && estopActive == false) {
        // Read one line of command from file (up to newline character)
        Cmd = gcFile.readStringUntil('\n');
        // Check if command starts with "X" (Cartesian move command)
        if (Cmd.substring(0, 1) == "X") {
          // Update current position before executing move
          updatePos();
          // Execute Cartesian move command from file (false=file mode, true=G-code)
          moveJ(Cmd, false, false, true);
        }
        // PRECALC'D CMD - not currently used, needs position handling
        // This section parses pre-calculated step format from SD card
        else {
          // Parse comma-delimited position indices from command string
          int i1 = Cmd.indexOf(',');
          int i2 = Cmd.indexOf(',', i1 + 1);
          int i3 = Cmd.indexOf(',', i2 + 1);
          int i4 = Cmd.indexOf(',', i3 + 1);
          int i5 = Cmd.indexOf(',', i4 + 1);
          int i6 = Cmd.indexOf(',', i5 + 1);
          int i7 = Cmd.indexOf(',', i6 + 1);
          int i8 = Cmd.indexOf(',', i7 + 1);
          int i9 = Cmd.indexOf(',', i8 + 1);
          int i10 = Cmd.indexOf(',', i9 + 1);
          int i11 = Cmd.indexOf(',', i10 + 1);
          int i12 = Cmd.indexOf(',', i11 + 1);
          int i13 = Cmd.indexOf(',', i12 + 1);
          int i14 = Cmd.indexOf(',', i13 + 1);
          int i15 = Cmd.indexOf(',', i14 + 1);
          int i16 = Cmd.indexOf(',', i15 + 1);
          int i17 = Cmd.indexOf(',', i16 + 1);
          int i18 = Cmd.indexOf(',', i17 + 1);
          int i19 = Cmd.indexOf(',', i18 + 1);
          int i20 = Cmd.indexOf(',', i19 + 1);
          int i21 = Cmd.indexOf(',', i20 + 1);
          int i22 = Cmd.indexOf(',', i21 + 1);
          int i23 = Cmd.indexOf(',', i22 + 1);
          // Extract step counts for J1-J9 (absolute steps needed for motion)
          int J1step = Cmd.substring(0, i1).toInt();
          int J2step = Cmd.substring(i1 + 1, i2).toInt();
          int J3step = Cmd.substring(i2 + 1, i3).toInt();
          int J4step = Cmd.substring(i3 + 1, i4).toInt();
          int J5step = Cmd.substring(i4 + 1, i5).toInt();
          int J6step = Cmd.substring(i5 + 1, i6).toInt();
          int J7step = Cmd.substring(i6 + 1, i7).toInt();
          int J8step = Cmd.substring(i7 + 1, i8).toInt();
          int J9step = Cmd.substring(i8 + 1, i9).toInt();
          // Extract motor direction bits for J1-J9 (1=backward, 0=forward)
          int J1dir = Cmd.substring(i9 + 1, i10).toInt();
          int J2dir = Cmd.substring(i10 + 1, i11).toInt();
          int J3dir = Cmd.substring(i11 + 1, i12).toInt();
          int J4dir = Cmd.substring(i12 + 1, i13).toInt();
          int J5dir = Cmd.substring(i13 + 1, i14).toInt();
          int J6dir = Cmd.substring(i14 + 1, i15).toInt();
          int J7dir = Cmd.substring(i15 + 1, i16).toInt();
          int J8dir = Cmd.substring(i16 + 1, i17).toInt();
          int J9dir = Cmd.substring(i17 + 1, i18).toInt();
          // Extract speed type character (s=seconds, m=mm/sec, p=percentage)
          String SpeedType = Cmd.substring(i18 + 1, i19);
          // Extract speed value (magnitude for speed calculation)
          float SpeedVal = Cmd.substring(i19 + 1, i20).toFloat();
          // Extract acceleration percentage (percentage of steps to accelerate)
          float ACCspd = Cmd.substring(i20 + 1, i21).toFloat();
          // Extract deceleration percentage (percentage of steps to decelerate)
          float DCCspd = Cmd.substring(i21 + 1, i22).toFloat();
          // Extract acceleration ramp rate (percentage per step increment)
          float ACCramp = Cmd.substring(i22 + 1).toFloat();
          // Call G-code motion function with all parsed parameters
          driveMotorsG(J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        }
      }
      // Close file after reading all commands
      gcFile.close();
      // Send robot position back to user after playback complete
      sendRobotPos();
    }

    // WRITE PRE-CALC'D MOVE TO SD CARD - Pre-calculate motion and store to file
    else if (function == "WG") {
      // Declare direction variables for each motor (1=backward, 0=forward)
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      // Initialize axis fault flags for limit checking
      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      // Sum of all axis faults (0 = no errors, >0 = limit violation)
      int TotalAxisFault = 0;

      // String to store formatted motion command for writing to file
      String info;

      // Parse command parameter positions (location of each parameter in string)
      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");
      int fileStart = inData.indexOf("Fn");

      // Extract target X, Y, Z Cartesian position from command
      xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
      // Extract target orientation (Rz, Ry, Rx rotation angles in degrees)
      xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      // Extract external axis target positions (J7, J8, J9)
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      // Extract speed type (s=seconds, m=mm/sec, p=percentage speed)
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      // Extract speed value (seconds, mm per second, or percentage)
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      // Extract acceleration profile percentage (% of steps for acceleration phase)
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      // Extract deceleration profile percentage (% of steps for deceleration phase)
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      // Extract acceleration ramp rate (rate of change of velocity)
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      // Extract corner rounding radius (smooth corners in spline motion)
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      // Extract wrist configuration (determines IK solution orientation)
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      // Extract loop mode string for J1-J6 motion behavior
      String LoopMode = inData.substring(LoopModeStart + 2, fileStart);
      // Extract output filename for pre-calculated command storage
      String filename = inData.substring(fileStart + 2);

      // Parse loop mode for each joint (0=linear, 1=modulo for continuous rotation)
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      // Solve inverse kinematics to find joint angles for target position
      SolveInverseKinematics();

      // Calculate destination motor steps for each joint from IK solution
      int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
      int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
      int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
      int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
      int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
      int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;
      // Calculate steps for external axes (J7, J8, J9)
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;

      // Calculate step deltas (current position to target position) for each axis
      int J1StepDelta = J1MasterStep - J1FutureMasterStep;
      int J2StepDelta = J2MasterStep - J2FutureMasterStep;
      int J3StepDelta = J3MasterStep - J3FutureMasterStep;
      int J4StepDelta = J4MasterStep - J4FutureMasterStep;
      int J5StepDelta = J5MasterStep - J5FutureMasterStep;
      int J6StepDelta = J6MasterStep - J6FutureMasterStep;
      int J7StepDelta = J7MasterStep - J7futStepM;
      int J8StepDelta = J8MasterStep - J8futStepM;
      int J9StepDelta = J9MasterStep - J9futStepM;

      // Update current step position to target (commit motion to memory)
      J1MasterStep = J1FutureMasterStep;
      J2MasterStep = J2FutureMasterStep;
      J3MasterStep = J3FutureMasterStep;
      J4MasterStep = J4FutureMasterStep;
      J5MasterStep = J5FutureMasterStep;
      J6MasterStep = J6FutureMasterStep;
      J7MasterStep = J7futStepM;
      J8MasterStep = J8futStepM;
      J9MasterStep = J9futStepM;

      // Determine motor directions based on step differences (1=backward, 0=forward)
      J1dir = (J1StepDelta <= 0) ? 1 : 0;
      J2dir = (J2StepDelta <= 0) ? 1 : 0;
      J3dir = (J3StepDelta <= 0) ? 1 : 0;
      J4dir = (J4StepDelta <= 0) ? 1 : 0;
      J5dir = (J5StepDelta <= 0) ? 1 : 0;
      J6dir = (J6StepDelta <= 0) ? 1 : 0;
      J7dir = (J7StepDelta <= 0) ? 1 : 0;
      J8dir = (J8StepDelta <= 0) ? 1 : 0;
      J9dir = (J9StepDelta <= 0) ? 1 : 0;

      // Check if requested position is within J1 axis limits
      if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
        J1axisFault = 1;
      }
      // Check if requested position is within J2 axis limits
      if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
        J2axisFault = 1;
      }
      // Check if requested position is within J3 axis limits
      if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
        J3axisFault = 1;
      }
      // Check if requested position is within J4 axis limits
      if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
        J4axisFault = 1;
      }
      // Check if requested position is within J5 axis limits
      if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
        J5axisFault = 1;
      }
      // Check if requested position is within J6 axis limits
      if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
        J6axisFault = 1;
      }
      // Check if requested position is within J7 (external axis) limits
      if ((J7dir == 1 and (J7MasterStep + J7StepDelta > J7StepRange)) or (J7dir == 0 and (J7MasterStep - J7StepDelta < 0))) {
        J7axisFault = 1;
      }
      // Check if requested position is within J8 (external axis) limits
      if ((J8dir == 1 and (J8MasterStep + J8StepDelta > J8StepRange)) or (J8dir == 0 and (J8MasterStep - J8StepDelta < 0))) {
        J8axisFault = 1;
      }
      // Check if requested position is within J9 (external axis) limits
      if ((J9dir == 1 and (J9MasterStep + J9StepDelta > J9StepRange)) or (J9dir == 0 and (J9MasterStep - J9StepDelta < 0))) {
        J9axisFault = 1;
      }
      // Sum all axis fault flags (0=no errors, >0 = some axes have limit violations)
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

      // If no axis limit errors and kinematics valid, write pre-calculated command to file
      if (TotalAxisFault == 0 && KinematicError == 0) {
        // Format motion command as comma-delimited string: steps, directions, speeds, accel, ramp
        info = String(abs(J1StepDelta)) + "," + String(abs(J2StepDelta)) + "," + String(abs(J3StepDelta)) + "," + String(abs(J4StepDelta)) + "," + String(abs(J5StepDelta)) + "," + String(abs(J6StepDelta)) + "," + String(abs(J7StepDelta)) + "," + String(abs(J8StepDelta)) + "," + String(abs(J9StepDelta)) + "," + String(J1dir) + "," + String(J2dir) + "," + String(J3dir) + "," + String(J4dir) + "," + String(J5dir) + "," + String(J6dir) + "," + String(J7dir) + "," + String(J8dir) + "," + String(J9dir) + "," + String(SpeedType) + "," + String(SpeedVal) + "," + String(ACCspd) + "," + String(DCCspd) + "," + String(ACCramp);
        // Write formatted command string to SD card file
        writeSD(filename, info);
        // Send current robot position back to user
        sendRobotPos();
      } else if (KinematicError == 1) {
        // Kinematics error - target position unreachable
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        // Reset alarm code
        Alarm = "0";
      } else {
        // Axis limit error - report which axes exceeded limits (9-bit binary code)
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        // Reset alarm code
        Alarm = "0";
      }

      // Clear input buffer after command complete
      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE C (Cirlce) ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "MC") {
      // MOVE CIRCLE - Circular interpolated motion in Cartesian space
      // Interpolates along circular arc through intermediate point to final point

      // Motor direction control bits for each axis (1=backward, 0=forward)
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      // Axis fault flags: set to 1 if target position exceeds joint limits
      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      // Note: J7, J8, J9 faults not tracked for circle motion (6-axis only)
      // Status flag for alarm reporting (0 = no error)
      int TotalAxisFault = 0;

      // Alarm code string for status reporting to user
      String Alarm = "0";
      // Current distance to waypoint for motion planning
      float curWayDis;
      // Speed in microseconds for timing calculations (converted from user units)
      float speedSP;
      // Vector components for calculating distances and directions in Cartesian space
      float Xvect;
      float Yvect;
      float Zvect;
      // Calculated step gap (delay in microseconds between stepper pulses)
      float calcStepGap;
      // Rotation angle in radians for arc waypoint calculation
      float theta;
      // Circle direction: 1=clockwise, -1=counterclockwise based on arc angle
      int Cdir;
      // Rotation axis (unit vector) used for 3D arc rotation calculations
      float axis[3];
      // Temporary storage for axis calculations during normalization
      float axisTemp[3];
      // Starting vector for arc rotation (from center to start point)
      float startVect[3];
      // 3x3 rotation matrix for applying rotations to arc coordinates
      float Rotation[3][3];
      // Destination point after rotation applied
      float DestPt[3];
      // Quaternion components (a,b,c,d) used in Euler-Rodrigues formula for 3D rotation
      float a;
      float b;
      float c;
      float d;
      // Cached products of quaternion components for rotation matrix calculation (optimization)
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;

      // Parse command parameter positions (locations of parameters in input string)
      // Circle center position parameters (Cx, Cy, Cz)
      int xStart = inData.indexOf("Cx");
      int yStart = inData.indexOf("Cy");
      int zStart = inData.indexOf("Cz");
      // Orientation parameters (Rz, Ry, Rx angles in degrees)
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      // Arc start point on circle (Bx, By, Bz = "B" for beginning)
      int xMidIndex = inData.indexOf("Bx");
      int yMidIndex = inData.indexOf("By");
      int zMidIndex = inData.indexOf("Bz");
      // Arc end point on circle (Px, Py, Pz = "P" for point)
      int xEndIndex = inData.indexOf("Px");
      int yEndIndex = inData.indexOf("Py");
      int zEndIndex = inData.indexOf("Pz");
      // Tool offset (Tr = tool rotation/transformation)
      int tStart = inData.indexOf("Tr");
      // Speed parameters (S = speed value, Ac = acceleration, Dc = deceleration)
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      // Ramp rate (Rm = ramp mode/rate)
      int RmStart = inData.indexOf("Rm");
      // Wrist configuration and loop mode
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      // Extract circle center point coordinates (beginning point of circle definition)
      float xBeg = inData.substring(xStart + 2, yStart).toFloat();
      float yBeg = inData.substring(yStart + 2, zStart).toFloat();
      float zBeg = inData.substring(zStart + 2, rzStart).toFloat();
      // Extract starting orientation (circle center orientation)
      float rzBeg = inData.substring(rzStart + 2, ryStart).toFloat();
      float ryBeg = inData.substring(ryStart + 2, rxStart).toFloat();
      float rxBeg = inData.substring(rxStart + 2, xMidIndex).toFloat();
      // Extract arc start point (first point on the circle's arc)
      float xMid = inData.substring(xMidIndex + 2, yMidIndex).toFloat();
      float yMid = inData.substring(yMidIndex + 2, zMidIndex).toFloat();
      float zMid = inData.substring(zMidIndex + 2, xEndIndex).toFloat();
      // Extract arc end point (final point on the circle's arc)
      float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
      float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
      float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();
      // Extract tool transformation (typically tool offset or rotation angle)
      xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
      // Extract speed type character (s=seconds, m=mm/sec, p=percentage)
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      // Extract speed value (magnitude in selected units)
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      // Extract acceleration percentage (% of motion to accelerate)
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      // Extract deceleration percentage (% of motion to decelerate)
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      //calc vector from start point of circle (mid) to center of circle (beg)
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;
      //get radius - distance from first point (center of circle) to second point (start point of circle)
      float Radius = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);

      //set center coordinates of circle to first point (beg) as this is the center of our circle
      float Px = xBeg;
      float Py = yBeg;
      float Pz = zBeg;

      //define start vetor (mid) point is start of circle
      startVect[0] = (xMid - Px);
      startVect[1] = (yMid - Py);
      startVect[2] = (zMid - Pz);
      //get vectors from center of circle to  mid target (start) and end target then normalize
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;
      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;
      //get cross product of vectors b & c than apply to axis matrix
      float CrossX = (vect_By * vect_Cz) - (vect_Bz * vect_Cy);
      float CrossY = (vect_Bz * vect_Cx) - (vect_Bx * vect_Cz);
      float CrossZ = (vect_Bx * vect_Cy) - (vect_By * vect_Cx);
      axis[0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx, 2) + pow(vect_Cy, 2) + pow(vect_Bz, 2)) * sqrt(pow(vect_Cx, 2) + pow(vect_Cy, 2) + pow(vect_Cz, 2))));
      //get arc degree
      float ABdegrees = degrees(BCradians);
      //get direction from angle
      if (ABdegrees > 0) {
        Cdir = 1;
      } else {
        Cdir = -1;
      }

      //get circumference and calc way pt gap
      float lineDist = 2 * 3.14159265359 * Radius;
      float wayPts = lineDist / linWayDistSP;

      float wayPerc = 1 / wayPts;
      //cacl way pt angle
      float theta_Deg = ((360 * Cdir) / (wayPts));

      //determine steps
      int HighStep = lineDist / .05;
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.75;
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.75;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        float overclockPerc = speedSP / zeroTOTtime;
        calcStepGap = zeroStepGap * overclockPerc;
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

      //set starting delsy
      float curDelay = calcACCstartDel;

      // Initialize rotation angle for first waypoint (start at first incremental step)
      float cur_deg = theta_Deg;

      // Reset encoder fault tracking before motion execution
      resetEncoders();

      // Iterate through each waypoint along the circular arc
      // i: waypoint counter from 1 to wayPts
      for (int i = 1; i <= wayPts; i++) {

        // Convert current angle to radians for 3D rotation mathematics
        theta = radians(cur_deg);
        // Calculate quaternion (a,b,c,d) components from rotation axis and theta
        // Using Euler-Rodrigues formula: q = [cos(θ/2), -sin(θ/2)*axis_x, -sin(θ/2)*axis_y, -sin(θ/2)*axis_z]
        // Quaternions avoid gimbal lock and enable smooth arbitrary axis rotation
        a = cos(theta / 2.0);
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
        // Pre-compute products of quaternion components for rotation matrix calculation (optimization)
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        // Build 3x3 rotation matrix from quaternion using standard formula
        // R = [[aa+bb-cc-dd,  2(bc+ad),    2(bd-ac)  ],
        //      [2(bc-ad),     aa+cc-bb-dd, 2(cd+ab)  ],
        //      [2(bd+ac),     2(cd-ab),    aa+dd-bb-cc]]
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        // Apply rotation matrix to start vector to get waypoint position on arc
        // Matrix-vector product: DestPt = Rotation * startVect
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        // Calculate step delay for this waypoint with acceleration/deceleration ramping
        // During acceleration phase: decrease delay to increase speed
        if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          // During deceleration phase: increase delay to decrease speed
          curDelay = curDelay + (DCCwayInc);
        } else {
          // During normal speed phase: maintain constant delay
          curDelay = calcStepGap;
        }

        // Shift waypoint back to original circle center coordinates (add center offset)
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        // Maintain constant orientation throughout arc motion (wrist stays fixed)
        xyzuvw_In[3] = rzBeg;
        xyzuvw_In[4] = ryBeg;
        xyzuvw_In[5] = rxBeg;

        // Solve inverse kinematics for this waypoint position
        SolveInverseKinematics();

        // Calculate target motor steps for each joint at this waypoint
        int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

        // Calculate step difference (delta) from current position to waypoint target for each joint
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;
        // External axes J7-J9 not used in circle motion (6-axis only)
        int J7StepDelta = 0;
        int J8StepDelta = 0;
        int J9StepDelta = 0;

        // Determine motor rotation direction based on step difference sign
        // ternary operator: if stepDif <= 0 then direction=1 (backward), else 0 (forward)
        J1dir = (J1StepDelta <= 0) ? 1 : 0;
        J2dir = (J2StepDelta <= 0) ? 1 : 0;
        J3dir = (J3StepDelta <= 0) ? 1 : 0;
        J4dir = (J4StepDelta <= 0) ? 1 : 0;
        J5dir = (J5StepDelta <= 0) ? 1 : 0;
        J6dir = (J6StepDelta <= 0) ? 1 : 0;
        // J7-J9 not moving during circle motion (direction = 0 = stopped)
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        // Check if J1 axis position would exceed limits
        if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
          J1axisFault = 1;
        }
        // Check if J2 axis position would exceed limits
        if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
          J2axisFault = 1;
        }
        // Check if J3 axis position would exceed limits
        if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
          J3axisFault = 1;
        }
        // Check if J4 axis position would exceed limits
        if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
          J4axisFault = 1;
        }
        // Check if J5 axis position would exceed limits
        if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
          J5axisFault = 1;
        }
        // Check if J6 axis position would exceed limits
        if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
          J6axisFault = 1;
        }
        // Sum all joint fault flags (0=valid move, >0=at least one joint limit violation)
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

        // Execute motion step only if no axis limits exceeded AND kinematics solution found
        if (TotalAxisFault == 0 && KinematicError == 0) {
          // Call low-level motion driver with absolute step counts and directions
          // driveMotorsL handles stepper pulse generation with defined delay between steps
          driveMotorsL(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
        } else if (KinematicError == 1) {
          // Kinematics solver failed - target position unreachable
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        } else {
          // Axis limit violation - report which axes failed (9-bit binary code J1-J9)
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          delay(5);
          Serial.println(Alarm);
        }

        // Increment angle for next waypoint (rotate further around arc)
        cur_deg += theta_Deg;
      }

      // Check encoders for collision/slip detection after motion completes
      checkEncoders();
      // Send current robot position back to user
      sendRobotPos();

      // Clear input buffer after command processed
      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE A (Arc) ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "MA" and flag == "") {
      // MOVE ARC - Continuous motion along arc path with motion control
      // Supports lookahead for trajectory optimization and smooth corner rounding

      if (rndTrue == true) {
        inData = rndData;
      }

      float curDelay;

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";
      float curWayDis;
      float speedSP;
      float Xvect;
      float Yvect;
      float Zvect;
      float calcStepGap;
      float theta;
      float axis[3];
      float axisTemp[3];
      float startVect[3];
      float Rotation[3][3];
      float DestPt[3];
      float a;
      float b;
      float c;
      float d;
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;
      // Parse Input Data for Parameter Positions --------------------------------
      int xMidIndex = inData.indexOf("X");
      int yMidIndex = inData.indexOf("Y");
      int zMidIndex = inData.indexOf("Z");
      int rzIndex = inData.indexOf("Rz");
      int ryIndex = inData.indexOf("Ry");
      int rxIndex = inData.indexOf("Rx");

      int xEndIndex = inData.indexOf("Ex");
      int yEndIndex = inData.indexOf("Ey");
      int zEndIndex = inData.indexOf("Ez");
      int tStart = inData.indexOf("Tr");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      updatePos();

      float xBeg = xyzuvw_Out[0];
      float yBeg = xyzuvw_Out[1];
      float zBeg = xyzuvw_Out[2];
      float rzBeg = xyzuvw_Out[3];
      float ryBeg = xyzuvw_Out[4];
      float rxBeg = xyzuvw_Out[5];


      float xMid = inData.substring(xMidIndex + 1, yMidIndex).toFloat();
      float yMid = inData.substring(yMidIndex + 1, zMidIndex).toFloat();
      float zMid = inData.substring(zMidIndex + 1, rzIndex).toFloat();

      float rz = inData.substring(rzIndex + 2, ryIndex).toFloat();
      float ry = inData.substring(ryIndex + 2, rxIndex).toFloat();
      float rx = inData.substring(rxIndex + 2, xEndIndex).toFloat();


      float RZvect = rzBeg - rz;
      float RYvect = ryBeg - ry;
      float RXvect = rxBeg - rx;

      float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
      float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
      float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();


      xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      //determine length between each point (lengths of triangle)
      Xvect = xEnd - xMid;
      Yvect = yEnd - yMid;
      Zvect = zEnd - zMid;
      float aDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      Xvect = xEnd - xBeg;
      Yvect = yEnd - yBeg;
      Zvect = zEnd - zBeg;
      float bDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;
      float cDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      //use lengths between each point (lengths of triangle) to determine radius
      float s = (aDist + bDist + cDist) / 2;
      float Radius = aDist * bDist * cDist / 4 / sqrt(s * (s - aDist) * (s - bDist) * (s - cDist));
      //find barycentric coordinates of triangle (center of triangle)
      float BCx = pow(aDist, 2) * (pow(bDist, 2) + pow(cDist, 2) - pow(aDist, 2));
      float BCy = pow(bDist, 2) * (pow(cDist, 2) + pow(aDist, 2) - pow(bDist, 2));
      float BCz = pow(cDist, 2) * (pow(aDist, 2) + pow(bDist, 2) - pow(cDist, 2));
      //find center coordinates of circle - convert barycentric coordinates to cartesian coordinates - dot product of 3 points and barycentric coordiantes divided by sum of barycentric coordinates
      float Px = ((BCx * xBeg) + (BCy * xMid) + (BCz * xEnd)) / (BCx + BCy + BCz);
      float Py = ((BCx * yBeg) + (BCy * yMid) + (BCz * yEnd)) / (BCx + BCy + BCz);
      float Pz = ((BCx * zBeg) + (BCy * zMid) + (BCz * zEnd)) / (BCx + BCy + BCz);
      //define start vetor
      startVect[0] = (xBeg - Px);
      startVect[1] = (yBeg - Py);
      startVect[2] = (zBeg - Pz);
      //get 3 vectors from center of circle to begining target, mid target and end target then normalize
      float vect_Amag = pow((pow((xBeg - Px), 2) + pow((yBeg - Py), 2) + pow((zBeg - Pz), 2)), .5);
      float vect_Ax = (xBeg - Px) / vect_Amag;
      float vect_Ay = (yBeg - Py) / vect_Amag;
      float vect_Az = (zBeg - Pz) / vect_Amag;
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;
      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;
      //get cross product of vectors a & c than apply to axis matrix
      float CrossX = (vect_Ay * vect_Bz) - (vect_Az * vect_By);
      float CrossY = (vect_Az * vect_Bx) - (vect_Ax * vect_Bz);
      float CrossZ = (vect_Ax * vect_By) - (vect_Ay * vect_Bx);
      axis[0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float ABradians = acos((vect_Ax * vect_Bx + vect_Ay * vect_By + vect_Az * vect_Bz) / (sqrt(pow(vect_Ax, 2) + pow(vect_Ay, 2) + pow(vect_Az, 2)) * sqrt(pow(vect_Bx, 2) + pow(vect_By, 2) + pow(vect_Bz, 2))));
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx, 2) + pow(vect_By, 2) + pow(vect_Bz, 2)) * sqrt(pow(vect_Cx, 2) + pow(vect_Cy, 2) + pow(vect_Cz, 2))));
      //get total degrees of both arcs
      float ABdegrees = degrees(ABradians + BCradians);
      //get arc length and calc way pt gap

      float anglepercent = ABdegrees / 360;
      float circumference = 2 * 3.14159265359 * Radius;
      float lineDist = circumference * anglepercent;
      float wayPts = lineDist / linWayDistSP;

      float wayPerc = 1 / wayPts;
      //cacl way pt angle
      float theta_Deg = (ABdegrees / wayPts);

      //determine steps
      int HighStep = lineDist / .05;
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.2;
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        float overclockPerc = speedSP / zeroTOTtime;
        calcStepGap = zeroStepGap * overclockPerc;
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

      //set starting delsy
      if (rndTrue == true) {
        curDelay = rndSpeed;
      } else {
        curDelay = calcACCstartDel;
      }


      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////

      resetEncoders();

      for (int i = 0; i <= wayPts - 1; i++) {

        theta = radians(cur_deg);
        //use euler rodrigues formula to find rotation vector
        a = cos(theta / 2.0);
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        ////DELAY CALC/////
        if (rndTrue == true) {
          curDelay = rndSpeed;
        } else if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          curDelay = curDelay + (DCCwayInc);
        } else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        float curWayPerc = wayPerc * i;
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg - (RZvect * curWayPerc);
        xyzuvw_In[4] = ryBeg - (RYvect * curWayPerc);
        xyzuvw_In[5] = rxBeg - (RXvect * curWayPerc);


        SolveInverseKinematics();

        //calc destination motor steps
        int J1FutureMasterStep = (JointAnglesInverseKinematic[0] + J1axisLimNeg) * J1StepsPerDegree;
        int J2FutureMasterStep = (JointAnglesInverseKinematic[1] + J2axisLimNeg) * J2StepDeg;
        int J3FutureMasterStep = (JointAnglesInverseKinematic[2] + J3axisLimNeg) * J3StepDeg;
        int J4FutureMasterStep = (JointAnglesInverseKinematic[3] + J4axisLimNeg) * J4StepDeg;
        int J5FutureMasterStep = (JointAnglesInverseKinematic[4] + J5axisLimNeg) * J5StepDeg;
        int J6FutureMasterStep = (JointAnglesInverseKinematic[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1StepDelta = J1MasterStep - J1FutureMasterStep;
        int J2StepDelta = J2MasterStep - J2FutureMasterStep;
        int J3StepDelta = J3MasterStep - J3FutureMasterStep;
        int J4StepDelta = J4MasterStep - J4FutureMasterStep;
        int J5StepDelta = J5MasterStep - J5FutureMasterStep;
        int J6StepDelta = J6MasterStep - J6FutureMasterStep;
        int J7StepDelta = 0;
        int J8StepDelta = 0;
        int J9StepDelta = 0;

        //determine motor directions
        J1dir = (J1StepDelta <= 0) ? 1 : 0;
        J2dir = (J2StepDelta <= 0) ? 1 : 0;
        J3dir = (J3StepDelta <= 0) ? 1 : 0;
        J4dir = (J4StepDelta <= 0) ? 1 : 0;
        J5dir = (J5StepDelta <= 0) ? 1 : 0;
        J6dir = (J6StepDelta <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1MasterStep + J1StepDelta > J1StepRange)) or (J1dir == 0 and (J1MasterStep - J1StepDelta < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2MasterStep + J2StepDelta > J2StepRange)) or (J2dir == 0 and (J2MasterStep - J2StepDelta < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3MasterStep + J3StepDelta > J3StepRange)) or (J3dir == 0 and (J3MasterStep - J3StepDelta < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4MasterStep + J4StepDelta > J4StepRange)) or (J4dir == 0 and (J4MasterStep - J4StepDelta < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5MasterStep + J5StepDelta > J5StepRange)) or (J5dir == 0 and (J5MasterStep - J5StepDelta < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6MasterStep + J6StepDelta > J6StepRange)) or (J6dir == 0 and (J6MasterStep - J6StepDelta < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsL(abs(J1StepDelta), abs(J2StepDelta), abs(J3StepDelta), abs(J4StepDelta), abs(J5StepDelta), abs(J6StepDelta), abs(J7StepDelta), abs(J8StepDelta), abs(J9StepDelta), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
        } else if (KinematicError == 1) {
          Alarm = "ER";
          if (splineTrue == false) {
            delay(5);
            Serial.println(Alarm);
          }
        } else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          if (splineTrue == false) {
            delay(5);
            Serial.println(Alarm);
          }
        }

        //increment angle
        cur_deg += theta_Deg;
      }
      checkEncoders();
      rndTrue = false;
      inData = "";  // Clear recieved buffer
      if (splineTrue == false) {
        sendRobotPos();
      }
      ////////MOVE COMPLETE///////////
    }

    else {
      Serial.print("Error: Command Not Found: ");
      Serial.println(function);
    }

    //shift cmd buffer
    inData = "";
    cmdBuffer1 = "";
    shiftCMDarray();
  }
}
