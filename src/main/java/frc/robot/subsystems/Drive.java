// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANFalconFactory;
import frc.lib.util.DriveSignal;
import frc.lib.util.motion_profiles.MotionProfileBase;
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
  MotionProfileBase selectedProfile;
  /**
   * true = BRAKE, false = COAST
   */
  boolean brakeMode = true;

  /**
   * If enabled, hold in current position on MP
   */
  boolean hold = false;
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

    // get MP status, if in MP mode load and execute
    leftProfileController.control();
    rightProfileController.control();

    if (profileEnabled) {
      // get value to pass to falcon (tell it to do MP or not)
      SetValueMotionProfile setOutputL = leftProfileController.getSetValue();
      SetValueMotionProfile setOutputR = rightProfileController.getSetValue();

      if (hold) {
        setOutputL = SetValueMotionProfile.Hold;
        setOutputR = SetValueMotionProfile.Hold;
      }

      leftMain.set(ControlMode.MotionProfile, setOutputL.value);
      rightMain.set(ControlMode.MotionProfile, setOutputR.value);
    }
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

  /**
   * Remember to call this before executing the profile
   * @param profile Motion Profile that will be executed when 
   * the robot is placed in MP mode.
   */
  public void selectProfile(MotionProfileBase profile) {
    selectedProfile = profile;
  }

  public void setProfileEnabled(boolean enable) {
    if (enable != profileEnabled && enable) { // false => true
      try {
        System.out.println("Starting Motion Profile");
        startMotionProfile(selectedProfile);
      } catch (Exception e) {
        DriverStation.reportError(
          "selectedProfile was null, select a profile before attempting to start one!",
          e.getStackTrace());
      }
    }
  }

  public void startMotionProfile(MotionProfileBase mp) {
    leftProfileController = new MotionProfileHelper(leftMain, mp.pointsLeft, mp.numPointsLeft, leftMain.getInverted());
    rightProfileController = new MotionProfileHelper(rightMain, mp.pointsRight, mp.numPointsRight, rightMain.getInverted());
  }

  public void stopDrive() {
    profileEnabled = false;
    setOpenLoop(DriveSignal.NEUTRAL);
    leftProfileController.reset();
    rightProfileController.reset();
  }

  private double inchesToTicks(double inches) {
    return inches*Constants.kDriveTicksPerInch;
  }
 
  // TODO: PATHWEAVER PARAMS
  // max vel = 1.0m/s
  // max vel = 1.0 m/s^2
  // wheel base = 0.572m
}
