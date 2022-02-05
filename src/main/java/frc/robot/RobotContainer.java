// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TankDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final Intake m_intake = new Intake();
  public final DriveTrain m_driveTrain = new DriveTrain();

  public final ShooterCommand m_ShooterCommand = new ShooterCommand(m_shooterSubsystem);
  public final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final IntakeSpin m_intakeSpin = new IntakeSpin(m_intake,true);
  public final TankDrive m_tankDriveCMD = new TankDrive(m_driveTrain);
  


  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrain.setDefaultCommand(m_tankDriveCMD);
    m_shooterSubsystem.setDefaultCommand(m_ShooterCommand);

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Command Chooser", m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("[RobotContainer::ConfigureButtonBindings] Configuring Button Bindings...");
    GenericHID m_logiController = new GenericHID(0);
    // a shoots, x spins intake forward, y spins intake backward

    JoystickButton A_BUTTON = new JoystickButton(m_logiController, 1);
    A_BUTTON.whenPressed(new ShooterCommand(m_shooterSubsystem));

    JoystickButton X_BUTTON = new JoystickButton(m_logiController, 3);
    X_BUTTON.whenPressed(new IntakeSpin(m_intake, true));

    JoystickButton Y_BUTTON = new JoystickButton(m_logiController, 4);
    Y_BUTTON.whenPressed(new IntakeSpin(m_intake, false));
    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //TrajectoryConfig config = new TrajectoryConfig(
    //    Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    //config.setKinematics(m_driveTrain.getKinematics());
    return m_chooser.getSelected();
  }
}
