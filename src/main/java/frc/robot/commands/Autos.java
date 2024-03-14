package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Commands;



public class Autos extends SequentialCommandGroup  {
  
  public Autos(DriveSubsystem drive, Elbow m_elbow, Shooter m_shooter, IntakeSubsystem m_intake) {
    {
      this.addCommands(
        new TankDrive(drive, 50, 50),
        //new ElbowPID(m_elbow, -11, 0),
        //new ShootNote(m_shooter),
        Commands.waitSeconds(4),
        new TankDrive(drive, 0, 0)

        
        //new IntakeNote(m_intake)
      );
  }
}
}

  // public Autos simpleAuto(DriveSubsystem drive) {
  //   {
  //     this.addCommands(
  //     new TankDrive(drive, 50, 50)
      
  //   );
  //   }
  //   return null;
    
  // }

  
  
  




    
    
    
    
    

