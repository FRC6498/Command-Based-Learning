// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;

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

    // Max Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Input Scaling
	public static final double leftStickInputScale = 0.5;
	public static final double rightStickInputScale = 0.5;
    public static int kDriveTicksPerMeter = 365955;
    // 1 rev = 2048 ticks
    // wheel dia = 5.85in
    // 26.67 motor rev = 1 wheel rev
    // 26.67*2048 = ticks per wheel rev
    // 1 wheel rev = 0.148m
    // 6.7 wheel rev = 1m
    // 
    /** */
    double distancePerPulse = (0.149 * Math.PI) / (double) 2048;
	public static double regularTurnReduction = 0;
	public static double kDriveSwivelReduction = 0;
	public static double ramseteB = 2;
	public static double ramseteZeta = 0.7;
	public static double robotTrackWidth = 0.572; // meters
	public static double gearRatio; // GEARING = 10
	public static double ksVolts = 0.654;
	public static double kvVoltSecondsPerMeter = 2.36;
    public static double kaVoltSecondsSquaredPerMeter = 0.284;
    public static double kPDriveVel = 2.37;
	public static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(robotTrackWidth);
    public static double maxVelocityMetersPerSecond = 3.0;
    public static double maxAccelMetersPerSecondSquared = 3.0;
}
