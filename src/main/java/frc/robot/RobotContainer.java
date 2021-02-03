// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.DriveSignal;
import frc.robot.commands.DriveOpenLoop;
import frc.robot.commands.DriveTestProfileCommand;
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
  private SendableChooser<Command> autoChooser;
  double throttle=0;
  double turn=0;
  boolean driveInverted;

  // Controllers
  XboxController driverHID;
  
  // Buttons
  JoystickButton driverA, driverB, driverX, driverY, driverLB;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate controllers
    driverHID = new XboxController(0);

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Consistency Test", new DriveTestProfileCommand());
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
    // Register Buttons
    driverA = new JoystickButton(driverHID, Button.kA.value);
    driverB = new JoystickButton(driverHID, Button.kB.value);
    driverX = new JoystickButton(driverHID, Button.kX.value);
    driverY = new JoystickButton(driverHID, Button.kY.value);
    driverLB = new JoystickButton(driverHID, Button.kBumperLeft.value);

    // Assign Triggers
    driverA.toggleWhenPressed(new DriveTestProfileCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void driverArcadeDrive() {
    throttle=0;
    turn=driverHID.getX(GenericHID.Hand.kLeft)*Constants.regularTurnReduction;
    if(driverHID.getTriggerAxis(GenericHID.Hand.kRight)>.05) {
        throttle=driverHID.getTriggerAxis(GenericHID.Hand.kRight);			
    }else if(driverHID.getTriggerAxis(GenericHID.Hand.kLeft)>.05) {
        throttle=-driverHID.getTriggerAxis(GenericHID.Hand.kLeft);
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
    if(driverHID.getStickButtonReleased(GenericHID.Hand.kLeft)){
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

  
}
