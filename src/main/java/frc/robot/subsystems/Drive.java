// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final WPI_TalonFX leftMain, leftFollow, rightMain, rightFollow;
  private final SpeedControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive diffdrive;
  private final AHRS gyro;
  private final DifferentialDriveOdometry odometry;

  /** Creates a new Drive. */
  public Drive() {
    leftMain = new WPI_TalonFX(Constants.kLeftMainFalconID);
    leftMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftFollow = new WPI_TalonFX(Constants.kLeftFollowFalconID);
    leftFollow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    rightMain = new WPI_TalonFX(Constants.kRightMainFalconID);
    rightMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    rightFollow = new WPI_TalonFX(Constants.kRightFollowFalconID);
    rightFollow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    leftMotors = new SpeedControllerGroup(leftMain, leftFollow);
    rightMotors = new SpeedControllerGroup(rightMain, rightFollow);
    
    diffdrive = new DifferentialDrive(leftMotors, rightMotors);
    
    gyro = new AHRS();

    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    leftMotors.setInverted(true);

    diffdrive.setSafetyEnabled(false);

    //leftFollow.follow(leftMain, FollowerType.PercentOutput);
    //rightFollow.follow(rightMain, FollowerType.PercentOutput);
    //leftMain.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getRotation2d(), 
      (leftMain.getSelectedSensorPosition()+leftFollow.getSelectedSensorPosition())/2, 
      (rightMain.getSelectedSensorPosition()+rightFollow.getSelectedSensorPosition())/2
    );
  }

  /**
   * 
   * @return The currently-estimated pose of the robot
   */
  public Pose2d getCurrentPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMain.getSelectedSensorVelocity(), 
      rightMain.getSelectedSensorVelocity()
    );
  }

  public void resetOdometry(Pose2d pose) 
  {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) 
  {
    diffdrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    diffdrive.feed();
  }

  public void resetEncoders()
  {
    leftMain.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance()
  {
    double leftavg = (leftMain.getSelectedSensorPosition() 
      + leftFollow.getSelectedSensorPosition()) / 2.0;
    double rightavg = (rightMain.getSelectedSensorPosition() 
      + rightFollow.getSelectedSensorPosition()) / 2.0;
    return (leftavg + rightavg)/2.0;
  }

  public void setMaxDriveOutput(double maxDrive)
  {
    diffdrive.setMaxOutput(maxDrive);
  }

  public void resetHeading()
  {
    gyro.reset();
  }

  public double getHeading()
  {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate()
  {
    return -gyro.getRate();
  }

  /**
   * Sets the output of the left side drive motors
   * @param speed The percent output to set
   */
  public void setLeftMotors(double speed) {
    //leftMain.set(ControlMode.PercentOutput, speed);
    leftMotors.set(speed);
  }

  /**
   * Sets the output of the right side drive motors
   * @param speed The percent output to set
   */
  public void setRightMotors(double speed) {
    //rightMain.set(ControlMode.PercentOutput, speed);
    rightMotors.set(speed);
  }


  // TODO: PATHWEAVER PARAMS
  // max vel = 1.0m/s
  // max vel = 1.0 m/s^2
  // wheel base = 0.572m
  /**
   * Stops all drive motors immediately
   */
  public void stopDrive() {
    leftMotors.stopMotor();
    rightMotors.stopMotor();
    //leftMain.set(ControlMode.PercentOutput, 0);
    //rightMain.set(ControlMode.PercentOutput, 0);
  }
}
