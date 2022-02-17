
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final Indexer m_indexer = new Indexer();
  public final Intake m_intake = new Intake();
  public final Elevator m_elevator = new Elevator();
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final Joystick l_stick = new Joystick(1);
  private final Joystick r_stick = new Joystick(0);

  private final GenericHID m_logiController = new GenericHID(2);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(l_stick.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(l_stick.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(r_stick.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //   m_drivetrainSubsystem,
    //   () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));


    m_elevator.setDefaultCommand(new ElevatorCommand(
      m_elevator,
      () -> m_logiController.getRawAxis(1),
      () -> m_logiController.getRawAxis(5)
    ));
    
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

    // Elevator uses logitech controller and everything else uses joystick
    // a shoots, x spins intake forward, y spins intake backward

    JoystickButton RIGHT_STICK_BUTTON_1 = new JoystickButton(r_stick, 1);
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(m_shooterSubsystem));

    JoystickButton LEFT_STICK_BUTTON_1 = new JoystickButton(l_stick, 1);
    LEFT_STICK_BUTTON_1.whenHeld(new IntakeSpin(m_intake, true));

    JoystickButton LEFT_STICK_BUTTON_3 = new JoystickButton(l_stick, 3);
    LEFT_STICK_BUTTON_3.whenHeld(new IntakeSpin(m_intake, false));

    JoystickButton RIGHT_STICK_BUTTON_3 = new JoystickButton(r_stick, 3);
    RIGHT_STICK_BUTTON_3.whenHeld(new IndexerCommand(m_indexer)); 
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
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
