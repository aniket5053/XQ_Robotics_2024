// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final Shooter m_shooter = new Shooter();
  public  Elbow m_elbow = new Elbow();
  public Elevator m_elevator = new Elevator();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


 // A chooser for autonomous commands
  private final Command m_simpleAuto = new Autos(m_robotDrive, m_elbow, m_shooter, m_intake);
    SendableChooser<Command> m_chooser = new SendableChooser<>();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
  //       // A split-stick arcade command, with forward/backward controlled by the left
  //       // hand, and turning controlled by the right.
  //       // new DefaultDrive(
  //       //     m_robotDrive,
  //       //     () -> -m_driverController.getLeftY() ,
  //       //     () -> m_driverController.getRightX()));

        new DefaultDrive(
            m_robotDrive,
            () -> -m_driverController.getLeftY() ,
            () -> m_driverController.getRightX()));

    

        new JoystickButton(m_operatorController, Button.kLeftBumper.value).debounce(0.1)
            .onTrue(
              new ElbowManual(
              m_elbow, 
              // If polarity is reversed use this:
              () -> -m_operatorController.getRightY())
              //() -> m_operatorController.getLRighteftY())
    );

     new JoystickButton(m_driverController, Button.kLeftBumper.value).debounce(0.1)
            .onTrue(
              new DefaultDrive(
            m_robotDrive,
            () -> m_driverController.getLeftY() ,
            () -> -m_driverController.getRightX()))
            .onFalse(new DefaultDrive(
            m_robotDrive,
            () -> -m_driverController.getLeftY() ,
            () -> m_driverController.getRightX()));
  
  



     // Put the chooser on the dashboard
     SmartDashboard.putData("Autonomous", m_chooser);
      m_chooser.setDefaultOption("SimpleAuto", m_simpleAuto);
      SmartDashboard.putData(m_elbow);
      SmartDashboard.putData(m_robotDrive);

  }

 


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  
    //Shoot note
    // new JoystickButton(m_operatorController, 2)
    // .whileTrue(new ShootNote(m_shooter));

     //Shoot subwoofer
    new JoystickButton(m_operatorController, Button.kY.value)
    .onTrue(new ElbowPID(m_elbow, 13, 0.0)
    .alongWith(new ShootNote(m_shooter))
    );

    // //Take out stuck note (FOR TESTING ONLY)
    new JoystickButton(m_operatorController, Button.kX.value)
    .whileTrue(new EjectNote(m_intake));

    // // Intake note with harvestor
    new JoystickButton(m_operatorController, Button.kB.value)
    .whileTrue(new IntakeNote(m_intake));

    //defense
     new POVButton(m_operatorController, 0)
    .onTrue(new ElbowPID(m_elbow, 30, 0.0)
    .alongWith(new StopShooter(m_shooter)));

    //home
     new POVButton(m_operatorController, 180)
    .onTrue(new ElbowPID(m_elbow, 0, 0.0)
    .alongWith(new StopShooter(m_shooter)));

  

    new JoystickButton(m_operatorController, Button.kStart.value)
    .onTrue(new ResetElbow(m_elbow));

    //shoot podium
     new JoystickButton(m_operatorController, Button.kA.value)
    .onTrue(new ElbowPID(m_elbow,25 , 0.0)
    .alongWith(new ShootNote(m_shooter)));

    //amp 
      new POVButton(m_operatorController, 90)
    .onTrue(new ElbowPID(m_elbow,40 , 0.0)
    .alongWith(new ElbowAmp(m_shooter)));

    //climber up
    new POVButton(m_operatorController, 270)
    .whileTrue(new climbereasy(m_elevator, 0.75));
    //.onTrue(new ClimberVeloCmd(m_elevator, ClimberConstants.kUpSpeed));

    //climber down
      new JoystickButton(m_operatorController, Button.kRightBumper.value)
          .whileTrue(new climbereasy(m_elevator, -0.75));

      //.onTrue(new ClimberVeloCmd(m_elevator,ClimberConstants.kDownSpeed));
 
            
  

    


    

    

    // new POVButton(m_operatorController, 0)
    // .whileTrue(new ElbowAmp(m_elbow, m_shooter, m_intake));

    // new JoystickButton(m_operatorController, Button.kRightBumper.value)
    // .whileTrue(new ElbowSpeakerWoofer(m_elbow, m_shooter, m_intake));
      
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
     return m_chooser.getSelected();
   }
}