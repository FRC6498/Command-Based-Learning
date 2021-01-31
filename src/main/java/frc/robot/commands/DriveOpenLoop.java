// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveOpenLoop extends CommandBase {

  private final Drive drive;
  private final DoubleSupplier left, right;
  /** Creates a new TankDrive. */
  public DriveOpenLoop(Drive subsystem, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    left = leftSupplier;
    right = rightSupplier;
    drive = subsystem;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setOpenLoop(Robot.m_robotContainer.getDriveSignal());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
