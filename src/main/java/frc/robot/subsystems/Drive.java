// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANFalconFactory;
import frc.lib.util.DriveSignal;
import frc.lib.util.motion_profiles.MotionProfileHelper;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  // Subsystem Instance
  private static Drive instance = new Drive();
  // Hardware
  private TalonFX leftMain, leftFollow, rightMain, rightFollow;
  // Controllers
  MotionProfileHelper leftProfileController, rightProfileController;
  boolean profileEnabled = false;
  /**
   * true = BRAKE, false = COAST
   */
  boolean brakeMode = true;
  /** Creates a new Drive. */
  public Drive() {
    // falcons
    leftMain = CANFalconFactory.createFalcon(Constants.kLeftMainFalconID,
     true, NeutralMode.Brake, FeedbackDevice.IntegratedSensor, 0, false);
    leftFollow = CANFalconFactory.createFalcon(Constants.kLeftFollowFalconID, 
    true, NeutralMode.Brake, FeedbackDevice.IntegratedSensor, 0, false);
    rightMain = CANFalconFactory.createFalcon(Constants.kRightMainFalconID, 
    false, NeutralMode.Brake, FeedbackDevice.IntegratedSensor, 0, false);
    rightFollow = CANFalconFactory.createFalcon(Constants.kRightFollowFalconID, 
    false, NeutralMode.Brake, FeedbackDevice.IntegratedSensor, 0, false);

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);
  }

  public static Drive getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setOpenLoop(DriveSignal signal)
  {
    if (!profileEnabled) {
      setBrakeMode(signal.getBrakeMode());
      // left side is reversed, but reverseOutput doesnt invert percentvbus
      leftMain.set(ControlMode.PercentOutput, signal.getLeft());
      rightMain.set(ControlMode.PercentOutput, signal.getRight());
    }
  }

  public void setBrakeMode(boolean brakeMode) {
    if (brakeMode) {
      leftMain.setNeutralMode(NeutralMode.Brake);
      rightMain.setNeutralMode(NeutralMode.Brake);
    }
    else {
      leftMain.setNeutralMode(NeutralMode.Coast);
      rightMain.setNeutralMode(NeutralMode.Coast);
    }
  }

  private double inchesToTicks(double inches) {
    return inches*Constants.kDriveTicksPerInch;
  }
 
  // TODO: PATHWEAVER PARAMS
  // max vel = 1.0m/s
  // max vel = 1.0 m/s^2
  // wheel base = 0.572m
}
