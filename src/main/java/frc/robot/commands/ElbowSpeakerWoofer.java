// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shooter;


public class ElbowSpeakerWoofer extends Command {
  private final Elbow m_elbow;
  private final Shooter m_shooter;
  private final IntakeSubsystem m_intake;
  /** Creates a new ElbowSpeakerWoofer. */
  public ElbowSpeakerWoofer(Elbow elbow, Shooter shooter, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elbow = elbow;
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(elbow);

    

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shootNote();
    Timer.delay(0.5);
    m_elbow.elbowSpeakerWoofer();
    Timer.delay(0.5);
    m_intake.getNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
    m_elbow.home();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
