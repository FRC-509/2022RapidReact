
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Static instances of subsystems are declared here and made public to allow for ease of access outside this class.
  public static final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  public static final Indexer s_indexer = new Indexer();
  public static final Intake s_intake = new Intake();
  public static final Elevator s_elevator = new Elevator();
  public static final SwerveDrive s_swerveDrive = new SwerveDrive();

  // Input devices are declared and initialized.
  public static final Joystick l_stick = new Joystick(1);
  public static final Joystick r_stick = new Joystick(0);

  public static final GenericHID s_logiController = new GenericHID(2);

  // A drop-down menu on the Smart Dashboard, allowing for the switching of autonomous routines.
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default commands are set for subsystems that may require continous input during Tele-op.
    s_swerveDrive.setDefaultCommand(new DefaultDriveCommand(
      () -> modifyAxis(l_stick.getX()) * SDSConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(l_stick.getY()) * SDSConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(r_stick.getX()) * SDSConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    s_elevator.setDefaultCommand(new ElevatorCommand(
      () -> s_logiController.getRawAxis(1),
      () -> -s_logiController.getRawAxis(5)
    ));
    
    // The default option for the sendable chooser is set, and is thrown onto the Dashboard.
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("Command Chooser", m_chooser);
  }

  private void configureButtonBindings() {
    System.out.println("[RobotContainer::ConfigureButtonBindings] Configuring Button Bindings...");

    // The elevator uses the Logitech F310 controller and everything else uses one of the two Joysticks.

    JoystickButton RIGHT_STICK_BUTTON_1 = new JoystickButton(r_stick, 1);
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(() -> l_stick.getRawAxis(3)));
    // Button 1 on the right stick must be held down to shoot.

    JoystickButton LEFT_STICK_BUTTON_1 = new JoystickButton(l_stick, 1);
    LEFT_STICK_BUTTON_1.whenHeld(new IntakeSpin(true));

    JoystickButton LEFT_STICK_BUTTON_3 = new JoystickButton(l_stick, 3);
    LEFT_STICK_BUTTON_3.whenHeld(new IntakeSpin(false));

    JoystickButton LEFT_STICK_BUTTON_11 = new JoystickButton(l_stick, 11);
    LEFT_STICK_BUTTON_11.whenPressed(new InstantCommand(s_swerveDrive::resetYawAngle, s_swerveDrive));
    
    JoystickButton RIGHT_STICK_BUTTON_3 = new JoystickButton(r_stick, 3);
    RIGHT_STICK_BUTTON_3.whenHeld(new IndexerCommand()); 
  }

  // A deadbander utility function.
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

  // A function that deadbands the input axis value to 0.2, and returns the value. This is used for the default commands of subsystems that require continous input.
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
  // This function returns the command to be executed in autonomous mode.
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
