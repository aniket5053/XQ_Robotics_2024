// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;         //NavX code
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;           //NavX code
import frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port,  MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightMotor1Port,  MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port,  MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

      public RelativeEncoder  m_leftEncoder = m_leftLeader.getEncoder();
      public RelativeEncoder  m_rightEncoder = m_rightLeader.getEncoder();
    
  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);

  // The gyro sensor
   public AHRS m_gyro = new AHRS(SPI.Port.kMXP);
   

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    zeroHeading();
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);

    m_leftLeader.setSmartCurrentLimit(40);
    m_leftFollower.setSmartCurrentLimit(40);
    m_rightLeader.setSmartCurrentLimit(40);
    m_rightFollower.setSmartCurrentLimit(40);



    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  @Override
    public void periodic() {
       log();
    }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double rot, double fwd) {
    m_drive.arcadeDrive(rot, fwd);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((double)m_leftEncoder.getPosition() +(double)m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return m_rightEncoder.getPosition();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getGyroAngle(){
    return m_gyro.getAngle();
  }


  public void log(){
    SmartDashboard.putNumber("leftDistance", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("rightDistance", m_rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public double getLeftDistance(){
    return m_leftEncoder.getDistance();
  }
  public double getRightDistance(){
    return m_rightEncoder.getDistance();
  }

  public void setCoastMode() {
    m_leftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBreakMode() {
    m_leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void log(){
    SmartDashboard.putNumber("Gyro Angle", getRightDistance());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("leftDistance", getLeftDistance());
    SmartDashboard.putNumber("rightDistance", getLeftDistance());

  }
 


}