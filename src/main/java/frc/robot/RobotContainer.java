// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final Shooter m_shooter = new Shooter();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


  private final Command simpleAuto = Autos.simpleAuto(m_robotDrive);
  private final Command complexAuto = Autos.complexAuto(m_robotDrive);
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //double getAngle = m_robotDrive.getGyroAngle();
  //double getHeading = m_robotDrive.getHeading();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
   m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
            m_robotDrive,
            () -> -m_driverController.getLeftY() ,
            () -> -m_driverController.getRightX()));


  
     m_chooser.setDefaultOption("Simple Auto", simpleAuto);
     m_chooser.addOption("Complex Auto", complexAuto);

     // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    //SmartDashboard.putNumber("angle", getAngle);
    //SmartDashboard.putNumber("heading", getHeading);

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


    // Turn to 90 degrees when the 'A' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new TurnToAngle(90, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the 'B' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(5));

    
    new JoystickButton(m_operatorController, Button.kB.value)
    .whileTrue(new IntakeNote(m_intake));

    new JoystickButton(m_operatorController, Button.kX.value)
    .whileTrue(new EjectNote(m_intake));

    new JoystickButton(m_operatorController, Button.kA.value)
    .whileTrue(new ShootNote(m_shooter));
      
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