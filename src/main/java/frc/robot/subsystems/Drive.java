// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANFalconFactory;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  // Subsystem Instance
  private static Drive instance = new Drive();
  // Hardware
  private WPI_TalonFX leftMain, leftFollow, rightMain, rightFollow;
  // Controllers
  private SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMain, leftFollow), 
  rightMotors = new SpeedControllerGroup(rightMain, rightFollow);

  public DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  public RamseteController ramseteController = new RamseteController(
    Constants.ramseteB, 
    Constants.ramseteZeta
    );
  public Rotation2d rotation = new Rotation2d();
  public Pose2d robotPose = new Pose2d();
  public DifferentialDriveOdometry odometry;
  // TRACK WIDTH MEASUERED DISTANCE WHEEL CENTER TO WHEEL CENTER IN METERS
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.robotTrackWidth);
  public static AHRS gyro;

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

    leftMain.setSafetyEnabled(false);
    leftFollow.setSafetyEnabled(false);
    rightMain.setSafetyEnabled(false);
    rightFollow.setSafetyEnabled(false);

    leftMain.setInverted(false);
    leftFollow.setInverted(false);
    rightMain.setInverted(true);
    rightFollow.setInverted(true);

    gyro = new AHRS(Port.kMXP);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    
  }

  public static Drive getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Rotation2d gyroAngleRotation2d = Rotation2d.fromDegrees(getHeading());
    odometry.update(gyroAngleRotation2d, getDistanceMeters(true), getDistanceMeters(false));  
  }

  public void setOpenLoop(DriveSignal signal)
  {
    // left side is reversed, but reverseOutput doesnt invert percentvbus
    drive.tankDrive(signal.getLeft(), signal.getRight());
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
   * Resets odometry to specified pose
   * 
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public Pose2d getRobotPoseMeters() {
    return odometry.getPoseMeters();
  }

  public double getGyroAngle() {
    return -gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getHeading() {
    return getGyroAngle() % 360;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getEncoderVelocity(leftMain), 
      getEncoderVelocity(rightMain)
    );
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    leftMain.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);
  }

  public double getEncoderPosition(WPI_TalonFX falcon) {
    return falcon.getSelectedSensorPosition();
  }
  public double getEncoderVelocity(WPI_TalonFX falcon) {
    return falcon.getSelectedSensorVelocity();
  }

  /**
   * Polls encoder for position in ticks and converts to inches.
   * 
   * @return Inches turned by specified motor.
   */
  private double getDistance(WPI_TalonFX falcon) {
    double inches = ((getEncoderPosition(falcon) / Constants.gearRatio) / 2048) * (6 * Math.PI);
    return inches;
  }

  /**
   * 
   * @param left Are we getting this distance travelled by the left side?
   * @return Distance in meters travelled by the specified side
   */
  private double getDistanceMeters(boolean left) {
    if (left) {
      return getDistance(leftMain) * 0.0254; // 1 inch = 0.0254m
    } else {
      return getDistance(rightMain) * 0.0254; // 1 inch = 0.0254m
    }
  }
 
  // TODO: PATHWEAVER PARAMS
  // max vel = 1.0m/s
  // max acc = 1.0 m/s^2
  // wheel base = 0.572m
}
