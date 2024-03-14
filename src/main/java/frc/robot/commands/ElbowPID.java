// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.Elbow;
import static frc.robot.Constants.ElbowConstants.*;

public class ElbowPID extends Command {
  private final Elbow elbowSubsystem;
  private final SparkPIDController pidController;
  private  double setpoint;
  private double tolerance;
  private double elbowMaxVel;
  private double elbowMaxAcc;
  private boolean firstTime = true;
  /** Creates a new ElbowPID. */
  public ElbowPID(Elbow elbowSubsystem, double setpoint, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elbowSubsystem = elbowSubsystem;
    this.pidController = elbowSubsystem.getIntegratedSparkPID();
    this.pidController.setP(kP); 
    this.pidController.setI(kI);
    this.pidController.setD(kD);
    this.pidController.setFF(kFF);
    this.pidController.setOutputRange(kMinOutput, kMaxOutput);
    this.setpoint = setpoint;
    this.tolerance = tolerance;

    int smartMotionSlot = 0;
    this.pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
    this.pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);
    this.pidController.setSmartMotionMaxVelocity(ElbowConstants.kMaxVelDown, smartMotionSlot);
    this.pidController.setSmartMotionMaxAccel(ElbowConstants.kMaxAccDown, smartMotionSlot);

    addRequirements(elbowSubsystem);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstTime = true;
    this.pidController.setSmartMotionMaxVelocity(ElbowConstants.kMaxVelDown, smartMotionSlot);
    this.pidController.setSmartMotionMaxAccel(ElbowConstants.kMaxAccDown, smartMotionSlot);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            double currentAngle = elbowSubsystem.getEncoderDegrees();
        if (firstTime) {

            if(setpoint > currentAngle) 
                {elbowMaxVel = ElbowConstants.kMaxVelUp; elbowMaxAcc = ElbowConstants.kMaxAccUp;}
            else  {elbowMaxVel = ElbowConstants.kMaxVelDown; elbowMaxAcc = ElbowConstants.kMaxAccDown;}

            this.pidController.setSmartMotionMaxVelocity(elbowMaxVel, smartMotionSlot);
            this.pidController.setSmartMotionMaxAccel(elbowMaxAcc, smartMotionSlot);

            firstTime = false;
        }

        // Using +/- 180 as inicator for small move up and down
        if(setpoint == 180.0) setpoint = currentAngle + ElbowConstants.kSmallMoveDegrees;
        if(setpoint == -180.0) setpoint = currentAngle - ElbowConstants.kSmallMoveDegrees;

        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Elbow Tolerance", this.tolerance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      firstTime = true;
      elbowSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tolerance == 0) return false;       // hold elevator at cmded position until another command moves it
        else if (Math.abs((elbowSubsystem.getEncoderDegrees() - setpoint)) < this.tolerance)   // else if within tolerance end Command 
            {
                firstTime = true;
                return true;
            }
        return false;
  }
}
