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
import frc.lib.util.DriveSignal;
import frc.robot.commands.DriveOpenLoop;
import frc.robot.subsystems.Drive;
import frc.lib.util.Util;

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
  double throttle=0;
  double turn=0;
  boolean driveInverted;

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
    autoChooser.addOption("Slalom", "Slalom");
    autoChooser.addOption("Barrels", "BarrelDodge");
    SmartDashboard.putData(autoChooser);
    m_drive.setDefaultCommand(
      new DriveOpenLoop(m_drive)
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

  public void driverArcadeDrive() {
    throttle=0;
    turn=mDriver.getX(GenericHID.Hand.kLeft)*Constants.regularTurnReduction;
    if(mDriver.getTriggerAxis(GenericHID.Hand.kRight)>.05) {
        throttle=mDriver.getTriggerAxis(GenericHID.Hand.kRight);			
    }else if(mDriver.getTriggerAxis(GenericHID.Hand.kLeft)>.05) {
        throttle=-mDriver.getTriggerAxis(GenericHID.Hand.kLeft);
        turn=-turn;
    }else {
        throttle=0;
        turn=turn*Constants.kDriveSwivelReduction;
    }
    if(getDriveInverted()&&throttle!=0){
        //turn=turn;
        throttle=-throttle;
    }
  }

  public boolean getDriveInverted() {
    if(mDriver.getStickButtonReleased(GenericHID.Hand.kLeft)){
        driveInverted=!driveInverted;
        //if(driveInverted)CameraVision.setStreamMode(StreamMode.LimeMain);
        //else CameraVision.setStreamMode(StreamMode.USBMain);
    }
  
    return driveInverted;
    //System.out.println("turn: "+turn+" throttle: "+throttle);
  }

  public double getThrottle() {
      driverArcadeDrive();
      return throttle;
  }

  public double getTurn() {
      driverArcadeDrive();
      return turn;
  }


  public DriveSignal getDriveSignal() {
    boolean squareInputs=true;
    double xSpeed;
    double zRotation;
    
    driverArcadeDrive();
    

    xSpeed=throttle;

    zRotation = turn;
    

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }
    
    

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
        } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
        } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
        }
    }
    double m_rightSideInvertMultiplier = -1.0;

    leftMotorOutput=leftMotorOutput*1;
    rightMotorOutput=rightMotorOutput*m_rightSideInvertMultiplier;

    
// System.out.println("Rot:"+turn+" xSpeed: "+xSpeed+" Left: "+leftMotorOutput+ " right: "+rightMotorOutput);
    return new DriveSignal(leftMotorOutput,rightMotorOutput,false);
    
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
    return null;
  }
}
