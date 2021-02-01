// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.motion_profiles;

/**
 * Defines the requirements for a motion Profile.
 * It contains the list of MP points as well as 
 * the number of points for each side of the robot.
*/
public abstract class MotionProfileBase {
    public int numPointsLeft;
    public double [][]pointsLeft;
    public int numPointsRight;
    public double [][]pointsRight;
}
