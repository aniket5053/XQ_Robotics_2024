// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  public CANSparkMax m_topShooter; 
  public CANSparkMax m_bottomShooter ;
  //public RelativeEncoder  encoder = m_topShooter.getEncoder();

  /** Creates a new Shooter. */
  public Shooter() {
     m_topShooter = new CANSparkMax(ShooterConstants.k_TopShooterPort, MotorType.kBrushless);
     m_bottomShooter = new CANSparkMax(ShooterConstants.k_BottomShooterPort, MotorType.kBrushless);
     m_bottomShooter.setInverted(true);


  }

  public void shootNote(){
    m_topShooter.set(ShooterConstants.shooterTopSpeed);
    m_bottomShooter.set(ShooterConstants.shooterBottomSpeed);
  }

  

  public void ejectNote(){
        m_topShooter.set(ShooterConstants.ejectSpeed);
       m_bottomShooter.set(ShooterConstants.ejectSpeed);

  }

  public void ampNote(){
       m_topShooter.set(ShooterConstants.ampSpeed);
       m_bottomShooter.set(ShooterConstants.ampSpeed);
  }

  public void stop(){
    m_topShooter.set(0);
    m_bottomShooter.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCoastMode() {
    m_topShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_bottomShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBreakMode() {
    m_topShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_bottomShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
}
