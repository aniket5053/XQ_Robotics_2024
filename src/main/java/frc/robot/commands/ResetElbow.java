// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.Elbow;

public class ResetElbow extends Command {
  private final Elbow m_elbow;
  /** Creates a new ResetElbow. */
  public ResetElbow(Elbow elbow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elbow = elbow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elbow.resetElbowEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
