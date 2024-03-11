// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;          
import frc.robot.Constants.ElbowConstants;

public class Elbow extends SubsystemBase {
  public final CANSparkMax m_leftElbow = new CANSparkMax(ElbowConstants.k_leftElbowPort, MotorType.kBrushless);
  public final CANSparkMax m_rightElbow = new CANSparkMax(ElbowConstants.k_rightElbowPort, MotorType.kBrushless);
  public RelativeEncoder  m_leftElbowEncoder = m_leftElbow.getEncoder();
  public RelativeEncoder  m_rightElbowEncoder = m_leftElbow.getEncoder();

  /** Creates a new Elbow. */
  public Elbow() {
    m_rightElbow.follow(m_leftElbow, true);
    m_leftElbow.setInverted(true);
    m_rightElbow.setInverted(true);
    m_leftElbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElbowConstants.kReverseSoftLimit);
    m_rightElbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElbowConstants.kReverseSoftLimit);

    m_leftElbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ElbowConstants.kForwardSoftLimit);
    m_rightElbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ElbowConstants.kForwardSoftLimit);


    m_rightElbow.setSmartCurrentLimit(40);
    m_leftElbow.setSmartCurrentLimit(40);
    m_leftElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);


  }

  public void controlMove(double speed){
    m_leftElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftElbow.set(speed);
    //m_rightElbow.set(speed);
  }

  public void elbowUp(){
    m_leftElbow.set(ElbowConstants.upSpeed);
  }



  public void home(){
    while(getEncoderDegrees() > 0){
        elbowDown();
      }
      controlMove(0);

  }

  public void elbowDown(){
    m_leftElbow.set(ElbowConstants.downSpeed);
  }

  public void stop(){
    m_leftElbow.set(0);
    //m_rightElbow.set(0);
  }

  public void setCoastMode() {
    m_leftElbow.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightElbow.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBrakeMode() {
    m_leftElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightElbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }


  public void resetElbowEncoder(){
    m_leftElbowEncoder.setPosition(0);
    m_rightElbowEncoder.setPosition(0);
  }

  public SparkPIDController getIntegratedSparkPID(){
    return m_leftElbow.getPIDController();
  }

  public double getEncoderDegrees(){
    return m_leftElbowEncoder.getPosition();
  }

  // Moving the arm down increases the angle value
  public double getSmallDownAngle() {
    double temp = getEncoderDegrees();
   if ( (ElbowConstants.kMaxAngle -  temp) > ElbowConstants.smallDegrees)
      return (temp + ElbowConstants.smallDegrees );
   else return (ElbowConstants.kMaxAngle - 1.0) ;    // dont go past Max
}

// Moving the arm up decreases the angle value
public double getSmallUpAngle() {
  double temp = getEncoderDegrees();
  if ( (temp - 0) > ElbowConstants.smallDegrees)
      return (temp - ElbowConstants.smallDegrees );
  else return (0) ;          // dont go past min / start angle
 }

 public boolean isAtMax() {
  //        return m_ElbowAtMaxAngle.get();
          return  ( (m_leftElbowEncoder.getVelocity() > 0.01) && (this.getEncoderDegrees() > ElbowConstants.kForwardSoftLimit)   );
  
      }

  public boolean isAtMin() {
  //        return m_ElbowAtMaxAngle.get();
          return  ( (m_leftElbowEncoder.getVelocity() < -0.01) && (this.getEncoderDegrees() < ElbowConstants.kReverseSoftLimit)   );
  
      }     
  
    public void elbowSpeakerWoofer(){
      while(getEncoderDegrees() < 20){
        elbowUp();
      }
      controlMove(0);
    }
 

  public void log(){
    SmartDashboard.putNumber("Left Elbow Encoder degrees", m_leftElbowEncoder.getPosition());
        SmartDashboard.putNumber("RightElbow Encoder degrees", m_rightElbowEncoder.getPosition());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
