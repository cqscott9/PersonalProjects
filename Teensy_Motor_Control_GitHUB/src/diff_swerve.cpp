/******************************************************************************************
  AUTHOR: 

  PURPOSE: Implementation of differential swerve kinematics and sinusoidal trajectory 
           generation for end effector control.

  UPDATED: 12/05/2025
******************************************************************************************/

#include "diff_swerve.h"
#include <math.h>

/******************************************************************************************
                                  Gear Ratio Configuration:
******************************************************************************************/
// TODO: MEASURE AND SET THESE VALUES FOR YOUR PHYSICAL SYSTEM

// Drive gear ratio: (motor_rotation) / (end_effector_frontal_rotation)
// To measure: Command both motors to rotate 1 rad in SAME direction
//             Measure resulting end effector frontal plane displacement
//             DRIVE_GEAR_RATIO = 1.0 / measured_displacement
const float DRIVE_GEAR_RATIO = 1.078f;  // PLACEHOLDER - MUST BE MEASURED

// Steer gear ratio: (motor_rotation) / (end_effector_sagittal_rotation)
// To measure: Command both motors to rotate 1 rad in OPPOSITE directions
//             Measure resulting end effector sagittal plane rotation
//             STEER_GEAR_RATIO = 1.0 / measured_rotation
const float STEER_GEAR_RATIO = 0.625f;  // PLACEHOLDER - MUST BE MEASURED

/******************************************************************************************
                                  Trajectory Parameters:
******************************************************************************************/
// These are the DESIRED amplitudes at the END EFFECTOR (not motor level)
static const float DRIVE_AMPLITUDE = 3* PI/2;    // Amplitude for Frontal plane motion [rad]
static const float STEER_AMPLITUDE = PI;    // Amplitude for Sagittal plane motion [rad]
static const float DRIVE_FREQUENCY = 0.125f;     // Frequency for drive motion [Hz]
static const float STEER_FREQUENCY = 0.25f;    // Frequency for steer motion [Hz]
static const float STEER_PHASE_SHIFT = PI/4;  // Phase shift (π/2 rad = 90 degrees)
static const float DRIVE_OMEGA = 2.0f * PI * DRIVE_FREQUENCY; // Angular frequency [rad/s]
static const float STEER_OMEGA = 2.0f * PI * STEER_FREQUENCY;

// Control gains
static const float KP = 2.0f; // Position gain
static const float KD = 0.5f; // Damping gain

// Time tracking
static float elapsed_time = 0.0f; 


/******************************************************************************************
  FUNCTION: Forward kinematics - converts motor positions to end effector positions
  
  THEORY: 
    - Drive (Frontal): average of both motors, scaled by drive gear ratio
    - Steer (Sagittal): half the difference between motors, scaled by steer gear ratio
  
  The gear ratios convert from motor-space to end-effector-space:
    end_effector_pos = motor_pos / gear_ratio
  
  ARGS:
    motors: Array containing the two Motor objects
    
  RETURNS:
    DiffSwerveState: Current end effector state (positions and velocities)
******************************************************************************************/
DiffSwerveState forward_kinematics(Motor* motors) {
  DiffSwerveState state;
  
  // Position kinematics with gear ratio compensation
  // Sum of motors controls drive, divided by gear ratio to get end effector position
  state.drive_position = (motors[0].pos + motors[1].pos) / (2.0f * DRIVE_GEAR_RATIO);
  
  // Difference of motors controls steer, divided by gear ratio to get end effector angle
  state.steer_position = (motors[0].pos - motors[1].pos) / (2.0f * STEER_GEAR_RATIO);
  
  // Velocity kinematics (using motor velocities)
  state.drive_velocity = (motors[0].vel + motors[1].vel) / (2.0f * DRIVE_GEAR_RATIO);
  state.steer_velocity = (motors[0].vel - motors[1].vel) / (2.0f * STEER_GEAR_RATIO);
  
  return state;
}


/******************************************************************************************
  FUNCTION: Inverse kinematics - converts end effector positions to motor positions
  
  THEORY:
    To achieve desired end effector motion, motors must move through gear ratio:
      motor_pos = end_effector_pos * gear_ratio
    
    For differential coupling:
    - Motor 0 gets both drive and steer contributions added
    - Motor 1 gets drive contribution minus steer contribution
  
  ARGS:
    drive_pos: Desired position in Frontal plane [rad]
    steer_pos: Desired position in Sagittal plane [rad]
    motor0_pos: Pointer to output motor 0 position [rad]
    motor1_pos: Pointer to output motor 1 position [rad]
******************************************************************************************/
void inverse_kinematics(float drive_pos, float steer_pos, float* motor0_pos, float* motor1_pos) {
  // Scale end effector positions by gear ratios to get motor positions
  float drive_motor = drive_pos * DRIVE_GEAR_RATIO;
  float steer_motor = steer_pos * STEER_GEAR_RATIO;
  
  // Differential coupling
  *motor0_pos = drive_motor + steer_motor;
  *motor1_pos = drive_motor - steer_motor;
}


/******************************************************************************************
  FUNCTION: Command end effector position using inverse kinematics
  
  This is the main interface for controlling the differential swerve module.
  It converts desired end effector positions to individual motor commands.
  
  ARGS:
    motors: Array containing the two Motor objects
    drive_pos: Desired position in Frontal plane [rad]
    steer_pos: Desired position in Sagittal plane [rad]
    kp: Position gain
    kd: Damping gain
******************************************************************************************/
void command_end_effector_position(Motor* motors, float drive_pos, float steer_pos, float kp, float kd) {
  float motor0_pos, motor1_pos;
  
  // Convert end effector commands to motor commands
  inverse_kinematics(drive_pos, steer_pos, &motor0_pos, &motor1_pos);
  
  // Send position commands to individual motors
  motors[0].set_position(motor0_pos, kp, kd);
  motors[1].set_position(motor1_pos, kp, kd);
}


/******************************************************************************************
  FUNCTION: Update the sinusoidal trajectory for differential swerve end effector
  
  This generates independent sinusoidal trajectories for the drive (Frontal) and 
  steer (Sagittal) degrees of freedom, with a phase offset between them.
  
  Call in the deterministic loop with the time step.
  
  ARGS:
    motors: Array containing the two Motor objects
    dt: Time step in seconds
******************************************************************************************/
void update_diff_swerve_sinusoidal(Motor* motors, float dt) {
  
  elapsed_time += dt;
  
  // Generate sinusoidal trajectories for end effector
  float drive_position = DRIVE_AMPLITUDE * sin(DRIVE_OMEGA * elapsed_time);
  float steer_position = STEER_AMPLITUDE * sin(STEER_OMEGA * elapsed_time + STEER_PHASE_SHIFT);
  
  // Command the end effector position (inverse kinematics handled internally)
  command_end_effector_position(motors, drive_position, steer_position, KP, KD);
}