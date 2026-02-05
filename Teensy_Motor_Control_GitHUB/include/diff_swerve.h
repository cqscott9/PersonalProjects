/******************************************************************************************
  AUTHOR: peepee poopoo

  PURPOSE: differential swerve module control header files, maps end effector 
  commands (drive and steer) to individual motor positions using differential kinematics

  DIFFERENTIAL SWERVE KINEMATICS:
  - How the motor0 and motor1 positions work ---
    * Drive (frontal plane): Average of both motor positions
    * Steer (sagittal plane): Difference between motor positions
  
  FORWARD KINEMATICS:
    drive_position = (motor0_pos + motor1_pos) / 2
    steer_position = (motor0_pos - motor1_pos) / 2
  
  INVERSE KINEMATICS:
    motor0_pos = drive_position + steer_position
    motor1_pos = drive_position - steer_position

  UPDATED: 12/05/2025
******************************************************************************************/

#ifndef DIFF_SWERVE_H
#define DIFF_SWERVE_H

#include "motors.h"

/******************************************************************************************
                                  Data Structures:
******************************************************************************************/

struct DiffSwerveState {
  float drive_position;  // Position in Frontal plane (translation) [rad]
  float steer_position;  // Position in Sagittal plane (rotation) [rad]
  float drive_velocity;  // Velocity in Frontal plane [rad/s]
  float steer_velocity;  // Velocity in Sagittal plane [rad/s]
};

/******************************************************************************************
                                  Constants:
******************************************************************************************/

// TODO: MEASURE THESE VALUES FROM YOUR PHYSICAL SYSTEM
// Drive gear ratio: motor_rotation / end_effector_frontal_rotation
// To measure: rotate both motors SAME direction, measure end effector frontal displacement
extern const float DRIVE_GEAR_RATIO;

// Steer gear ratio: motor_rotation / end_effector_sagittal_rotation  
// To measure: rotate both motors OPPOSITE directions, measure end effector sagittal rotation
extern const float STEER_GEAR_RATIO;

/******************************************************************************************
                                  Function Prototypes:
******************************************************************************************/

// Forward kinematics: motor positions -> end effector positions
DiffSwerveState forward_kinematics(Motor* motors);

// Inverse kinematics: end effector positions -> motor positions
void inverse_kinematics(float drive_pos, float steer_pos, float* motor0_pos, float* motor1_pos);

// Update sinusoidal trajectory for differential swerve end effector
void update_diff_swerve_sinusoidal(Motor* motors, float dt);

// Command end effector position directly (uses inverse kinematics internally)
void command_end_effector_position(Motor* motors, float drive_pos, float steer_pos, float kp, float kd);

#endif // DIFF_SWERVE_H