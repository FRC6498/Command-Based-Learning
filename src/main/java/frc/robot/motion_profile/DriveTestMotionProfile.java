// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motion_profile;

import frc.lib.util.motion_profiles.MotionProfileBase;

/** Profile used for drivebase consistency testing. */
public class DriveTestMotionProfile extends MotionProfileBase {
    public int numPointsLeft = 0;
    public int numPointsRight;

    public double [][]PointsLeft = new double[][] {

    };

    public double [][]PointsRight = new double[][] {

    };
}