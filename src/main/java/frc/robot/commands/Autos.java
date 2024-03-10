package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;


public class Autos {
    public static Command simpleAuto(DriveSubsystem drive) {
    return new FunctionalCommand(
        // Reset encoders on command start
        drive::resetEncoders,
        // Drive forward while the command is executing
        () -> drive.arcadeDrive(0,900),
        // Stop driving at the end of the command
        interrupt -> drive.arcadeDrive(0, 0),
        // End the command when the robot's driven distance exceeds the desired value
        () -> drive.getRightEncoder() >= 60,
        // Require the drive subsystem
        drive);
  }

  public static Command complexAuto(DriveSubsystem drive) {

    return Commands.sequence(
    //move forward
       new FunctionalCommand(
        // Reset encoders on command start
        drive::resetEncoders,
        // Drive forward while the command is executing
        () -> drive.arcadeDrive(90, 0),
        // Stop driving at the end of the command
        interrupt -> drive.arcadeDrive(0, 0),
        // End the command when the robot's driven distance exceeds the desired value
        () -> drive.getAverageEncoderDistance() >= 60,
        // Require the drive subsystem
        drive),

        //try turning
        new FunctionalCommand(
        // Reset encoders on command start
        drive::resetEncoders,
        // Drive forward while the command is executing
        () -> drive.arcadeDrive(0, 75),
        // Stop driving at the end of the command
        interrupt -> drive.arcadeDrive(0, 0),
        // End the command when the robot's driven distance exceeds the desired value
        () -> drive.getAverageEncoderDistance() >= 10,
        // Require the drive subsystem
        drive));

        
  }


    
    
    
    }
    

