// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 4;
    public static final int kRightMotor2Port = 5;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 0.5;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0.10;

    public static final double kTurnP = 0.5;
    public static final double kTurnI = 0.5;
    public static final double kTurnD = 0.10;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
  }

  public static final class DriveStraightConstants {
    // these pid constants are not real, and will need to be tuned
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class IntakeConstants{
      public static final int kIntakePort = 10;
      public static final double intakeSpeed = 0.5;
      public static final double ejectSpeed = -0.5;

  }

  public static final class ShooterConstants{
    public static final int k_TopShooterPort = 11;
    public static final int k_BottomShooterPort = 12;
    public static final double shooterTopSpeed = 0.5;
    public static final double shooterBottomSpeed = 0.35;
    public static final double ejectSpeed = -0.5;
    public static final double ampSpeed = 0.3;
  }

  public static final class ElbowConstants{
    public static final int k_leftElbowPort = 20;
    public static final int k_rightElbowPort = 21;

    public static final double upSpeed = 0.5;
    public static final double downSpeed = -0.2;


    public static final double smallDegrees = 1;
    public static final double kScoreInSpeakerAngleWoofer = 20.0;     
    
        // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
    // ****    This is a Position based PID That uses a Trapazoidal Profile for movement
    public static final double kP = 5e-5; 
    public static final double kI = 1e-8;   //TODO: was 1e-6 TRIED 1E-7, OVERSHOOTS ON >90
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.000156; // tried to change - only got worse!
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    //       public static final double maxRPM = 5700;
    // Smart Motion Coefficients
    public static final int smartMotionSlot = 0;
    public static final double kMinVel = 0;      // rpm
    public static final double kMaxVelUp = 4000;   // rpm    // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxAccUp = 2500;             // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxVelDown = 3500;   // rpm    // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxAccDown = 2000;             // TODO: TA - Optimize - was 4000, start with 500 for testing    
    public static final double kAllowedErr = 0;

// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;


    public static final double kZeroOffset = 0.0;
    public static final double kStartAngle = 0.0;
    public static final double kHomeAngle = 0.0;
    public static final double kScoreInSpeakerFromPodiumAngle = 29.0;       //was 30  //TA TODO: 
    public static final double kScoreInSpeakerFromSubwooferAngle = 13.0;            //TA TODO: 
    public static final double kScoreInAmpAngle = 90.0;        // TA TODO: was 95 with old setup
    public static final double kScoreInTrapAngle = 10.0;        

    public static float kReverseSoftLimit = -1;
    public static float kForwardSoftLimit = 38;

    public static float kSmallMoveDegrees = 3;
    public static final double kMaxAngle = (kForwardSoftLimit-2);

    public static final double kTolerance = 1.50;   // only used when going to zero so loosen up
    public static final double kHoldStillTolerance = 0.0;          
    public static final double kAutoTolerance = 0.250;           // TODO TA - may have to optimize   



  }

  public static final class ClimberConstants{
    public static final int kLeftClimberMotorPort = 30;
    public static final int kRightClimberMotorPort = 31;

    public static final double kUpSpeed = 0.75;      // now 25:1 raise speed
    public static final double kDownSpeed = -0.75;

    // CanSparkMax Encoder native units is rotations. We convert to inches for the elevator
    // rotations * circumference of wheel (pi*d) / the gear ratio: 
    // d= sprocket diameter * pulley diameter, 1.757*1.041" and gear down is 5:1 
    // THere is a factor of 2 that I dont understand?!?
//    public static final double kEncoderRotation2Inches = 1.0 * Math.PI * (1.757 * 1.041) / 5  * 1.925061;
    public static final double kEncoderRotation2Inches = 0.1354;

    // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
// ****    This is a Position based PID That uses a Trapazoidal Profile for movement
//    public static final double kP = 0.00020; 
    public static final double kP = 0.000600;        // changed from 15:1 to 25:1 - need larger P  .00040 was way too slow
    public static final double kI = 0.000005;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
//       public static final double maxRPM = 5700;
// Smart Motion Coefficients
    public static final int smartMotionSlot = 0;
    public static final double kMinVel = 0;      // rpm
    public static final double kMaxVelUp = 40000;   // rpm    // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxAccUp = 20000;             // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxVelDown = 40000;   // rpm    // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kMaxAccDown = 20000;             // TODO: TA - Optimize - was 4000, start with 500 for testing
    public static final double kAllowedErr = 0;
    
// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;


    public static final double kZeroOffset = 0.0;
    public static final double kHomePosition = 0.0 ;       // Keep home very slightly above hard stop
    public static final double kClimbPosition = 15.0 ;       

    public static final double kTolerance = 0.5 ;      
    
    public static float kReverseSoftLimit = 0;
    public static float kForwardSoftLimit = 19;

    public static double kSmallMoveInches = 3;

  }
}

 
