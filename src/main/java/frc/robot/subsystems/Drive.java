// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private TalonFX leftMain, leftFollow, rightMain, rightFollow;
  /** Creates a new Drive. */
  public Drive() {
    leftMain = new TalonFX(Constants.kLeftMainFalconID);
    leftFollow = new TalonFX(Constants.kLeftFollowFalconID);
    rightMain = new TalonFX(Constants.kRightMainFalconID);
    rightFollow = new TalonFX(Constants.kRightFollowFalconID);

    leftFollow.follow(leftMain, FollowerType.PercentOutput);
    rightFollow.follow(rightMain, FollowerType.PercentOutput);

    leftMain.setInverted(true);
    
  }


  public void setLeftMotors(double speed) {
    leftMain.set(ControlMode.PercentOutput, speed);
  }

  public void setRightMotors(double speed) {
    rightMain.set(ControlMode.PercentOutput, speed);
  }

  public void stopDrive()
  {
    leftMain.set(ControlMode.PercentOutput, 0);
    rightMain.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
