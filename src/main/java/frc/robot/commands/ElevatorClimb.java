// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ClimberConstants.*;


public class ElevatorClimb extends Command {
  
  private final Elevator climberSubsystem;
//    private final PIDController pidController;
    private final SparkPIDController pidController;
    private final double setpoint;

    public ElevatorClimb(Elevator climberSubsystem, double maxVel, double maxAcc, double setpoint) {
        this.climberSubsystem = climberSubsystem;

        this.pidController = climberSubsystem.getIntegratedSparkPID();

        this.pidController.setP(kP); 
        this.pidController.setI(kI);
        this.pidController.setD(kD);
        this.pidController.setFF(kFF);
        this.pidController.setOutputRange(kMinOutput, kMaxOutput);
        this.setpoint = setpoint;

        int smartMotionSlot = 0;
        this.pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        this.pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        this.pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        this.pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);


        addRequirements(climberSubsystem);
    }

    @Override   
    public void initialize() {
//      System.out.println("ClimberPIDCmd started!");
//      pidController.reset();
    }

    @Override
    public void execute() {
//       pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    @Override
    public void end(boolean interrupted) {
 // TA 2/23/23 I think we need to comment this line out so climber wont just "fall down"
        //       climberSubsystem.setMotor(0);
//        System.out.println("ClimberPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return (Math.abs((climberSubsystem.getEncoderInches() - setpoint)) < kTolerance);        
    }
}
