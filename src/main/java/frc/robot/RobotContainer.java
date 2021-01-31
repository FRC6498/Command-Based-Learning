// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DriveOpenLoop;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive m_drive = new Drive();
  String pathJSON = "";
  private Trajectory trajectory;
  private SendableChooser<String> autoChooser;


  // Controllers
  XboxController mDriver, mOperator;

  // Buttons

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    trajectory = new Trajectory();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Trajectory Test", "DriveTest");
    autoChooser.addOption("Slalom", null);
    autoChooser.addOption("Barrels", null);
    SmartDashboard.putData(autoChooser);
    m_drive.setDefaultCommand(
      new DriveOpenLoop(
        m_drive, 
        () -> mDriver.getY(GenericHID.Hand.kLeft),
        () -> mDriver.getY(GenericHID.Hand.kRight))
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    mDriver = new XboxController(0);
    mOperator = new XboxController(1);
  }

 private void GetTrajectory(String pathname)
 {
   pathJSON = String.format("paths/%s.wpilib.json", pathname);
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException e) {
    DriverStation.reportError("Unable to open Trajectory: " + pathJSON, e.getStackTrace());
  }
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    GetTrajectory("DriveTest");

      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory, 
        m_drive::getCurrentPose, 
        new RamseteController(
          Constants.kRamseteB, 
          Constants.kRamseteZeta
        ), 
        new SimpleMotorFeedforward(
          Constants.ksVolts, 
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter
        ),
        Constants.kDriveKinematics, 
        m_drive::getWheelSpeeds, 
        new PIDController(Constants.driveVelocitykP, 0, 0), 
        new PIDController(Constants.driveVelocitykP, 0, 0), 
        m_drive::tankDriveVolts,
        m_drive
      );

      m_drive.resetOdometry(trajectory.getInitialPose());
      return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0,0));
  }
}
