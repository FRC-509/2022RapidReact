
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.*;
import frc.robot.commands.AimBot;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IndexerSpin;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private static final Indexer s_indexer = new Indexer();
  private static final Intake s_intake = new Intake();
  private static final Elevator s_elevator = new Elevator();
  private static final DriveTrainSubsystem s_drivetrainSubsystem = new DriveTrainSubsystem();

  private static final Joystick l_stick = new Joystick(1);
  private static final Joystick r_stick = new Joystick(0);

  private static final GenericHID s_logiController = new GenericHID(2);

  private static final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static ShooterSubsystem getShooter() {
    return s_shooterSubsystem;
  }

  public static Indexer getIndexer() {
    return s_indexer;
  }

  public static Intake getIntake() {
    return s_intake;
  }

  public static Elevator getElevator() {
    return s_elevator;
  }

  public static DriveTrainSubsystem getDriveTrainSubsystem() {
    return s_drivetrainSubsystem;
  }

  public static Joystick getLeftStick() {
    return l_stick;
  }

  public static Joystick getRightStick() {
    return r_stick;
  }

  public static GenericHID getUSBController() {
    return s_logiController;
  }

  // If button 3 on the left stick is held down, steer from the X angle offset reported from the LimeLight. Otherwise, use the right stick's Y axis. 
  private static double angularInputCalculator() {
    if (l_stick.getRawButton(3))
      return -Math.toRadians(LimeLightWrapper.getX()) * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    else
      return modifyAxis(r_stick.getX()) * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // If button 9 on the left stick is held down, drive relative to the robot. Otherwise, drive relative to the field.
    s_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      () -> modifyAxis(l_stick.getY()) * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(l_stick.getX()) * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> angularInputCalculator(),
      () -> l_stick.getRawButton(9)
    ));

    s_elevator.setDefaultCommand(new ElevatorCommand(
      () -> s_logiController.getRawAxis(1),
      () -> -s_logiController.getRawAxis(5)
    ));
    
    m_chooser.setDefaultOption("2-ball auto, use this", new TwoBall());
    m_chooser.addOption("3-ball auto, use this if you dare", new ThreeBall());
    m_chooser.addOption("5-ball auto, DO NOT USE unless THERE IS NO OTHER OPTION", new FiveBall());

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

    // The elevator uses the Logitech F310 controller and everything else uses one of the two Joysticks.

    JoystickButton RIGHT_STICK_BUTTON_1 = new JoystickButton(r_stick, 1);
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand( () -> 0.32 ));
    // Button 1 on the right stick must be held down to shoot.

    JoystickButton RIGHT_STICK_BUTTON_3 = new JoystickButton(r_stick, 3);
    RIGHT_STICK_BUTTON_3.whenHeld(new IndexerSpin( () -> 0.3d ));

    JoystickButton LEFT_STICK_BUTTON_1 = new JoystickButton(l_stick, 1);
    LEFT_STICK_BUTTON_1.whenHeld(new IntakeSpin( () -> 1.0d ));

    JoystickButton LEFT_STICK_BUTTON_2 = new JoystickButton(l_stick, 2);
    LEFT_STICK_BUTTON_2.whenHeld(new IntakeSpin( () -> -1.0d ));

    JoystickButton LEFT_STICK_BUTTON_4 = new JoystickButton(l_stick, 4);
    LEFT_STICK_BUTTON_4.whenHeld(new AimBot());

    JoystickButton LEFT_STICK_BUTTON_11 = new JoystickButton(l_stick, 11);
    LEFT_STICK_BUTTON_11.whenPressed(new InstantCommand(s_drivetrainSubsystem::zeroGyroscope, s_drivetrainSubsystem));
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
  
  public Command getAutonomousCommand() {
    /*TrajectoryBuilderWrapper trajectoryBuilderWrapper = new TrajectoryBuilderWrapper("paths/Unnamed.wpilib.json");
    
    return new SequentialCommandGroup(
      trajectoryBuilderWrapper.getPathFollowingCommand(),
      new AimBot(),
      new ShooterCommand(() -> 0.35).withTimeout(2.0d)
    );*/
    return m_chooser.getSelected();
  }
}
