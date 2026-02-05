/******************************************************************************************
  AUTHOR: Bruh

  PURPOSE: Header file for sinusoidal trajectory generation for differential swerve drive.

  UPDATED: 12/04/2025
******************************************************************************************/

#ifndef SINUSOIDAL_TRAJECTORY_H
#define SINUSOIDAL_TRAJECTORY_H

#include "motors.h"

/******************************************************************************************
                                  Function Prototypes:
******************************************************************************************/

// Update motor positions with sinusoidal trajectory
void update_sinusoidal_trajectory(Motor* motors, float dt);

#endif // SINUSOIDAL_TRAJECTORY_H