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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;         //NavX code
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;           //NavX code
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  public  CANSparkMax m_intake;
  /** Creates a new IntakeSubSystem. */

  public IntakeSubsystem() {
    m_intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

  }

  public void getNote() {
    m_intake.set(IntakeConstants.intakeSpeed);
  }

  public void ejectNote() {
    m_intake.set(IntakeConstants.ejectSpeed);
  }

  public void stop() {
    m_intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
