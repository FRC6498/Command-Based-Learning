// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
    * DRIVE
    */

    // Motor IDs
    public static final int kLeftMainFalconID = 1;
    public static final int kLeftFollowFalconID = 2;
    public static final int kRightMainFalconID = 3;
    public static final int kRightFollowFalconID = 4;

    // Encoder units to SI Conversion Factors
    public static final double FalconUnitsToMetersNoGearing = 1; // wheel circumference / 2048
    public static final double FalconVelocityToMetersPerSecondNoGearing = 1;
    public static final double driveGearing = 20.8;
    /**
     * Trajectory Constants
    */ 
    // TODO: TUNE TRAJECTORY PID
    // Feedforward/Feedback Gains

    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;

    // Max Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // RAMSETE parameters
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // DifferentialDriveKinematics
    public static final double kTrackWidthMeters = 0.54;
    public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(kTrackWidthMeters);

    // Input Scaling
	public static final double leftStickInputScale = 0.5;
	public static final double rightStickInputScale = 0.5;
	public static double driveVelocitykP = 0;
}
