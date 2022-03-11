
package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimbotAndShoot;
//import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;

import java.nio.file.Path;
import java.util.List;
import java.util.function.DoubleSupplier;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final PIDController xController = new PIDController(1.5, 0, 0);
  public static final PIDController yController = new PIDController(1.5, 0, 0);
  public static final ProfiledPIDController thetaController = new ProfiledPIDController(3.0d, 0, 0, DrivetrainSubsystem.kThetaControllerConstraints);
  public static final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  public static final Indexer s_indexer = new Indexer();
  public static final Intake s_intake = new Intake();
  public static final Elevator s_elevator = new Elevator();
  public static final DrivetrainSubsystem s_drivetrainSubsystem = new DrivetrainSubsystem();

  public static final Joystick l_stick = new Joystick(1);
  public static final Joystick r_stick = new Joystick(0);

  public static final GenericHID s_logiController = new GenericHID(2);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private static Trajectory trajectory = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    s_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      () -> -modifyAxis(l_stick.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-l_stick.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-r_stick.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    s_elevator.setDefaultCommand(new ElevatorCommand(
      () -> s_logiController.getRawAxis(1),
      () -> -s_logiController.getRawAxis(5)
    ));
    
    m_chooser.setDefaultOption("Autonomous Command", null);

    SmartDashboard.putData("Command Chooser", m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Elevator uses logitech controller and everything else uses joystick
    // a shoots, x spins intake forward, y spins intake backward

    JoystickButton RIGHT_STICK_BUTTON_1 = new JoystickButton(r_stick, 1);
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(true));

    JoystickButton RIGHT_STICK_BUTTON_2 = new JoystickButton(r_stick, 2);
    RIGHT_STICK_BUTTON_2.whenHeld(new ShooterCommand(false));

    JoystickButton LEFT_STICK_BUTTON_1 = new JoystickButton(l_stick, 1);
    LEFT_STICK_BUTTON_1.whenHeld(new IntakeSpin(true));

    JoystickButton LEFT_STICK_BUTTON_3 = new JoystickButton(l_stick, 3);
    LEFT_STICK_BUTTON_3.whenHeld(new IntakeSpin(false));

    JoystickButton RIGHT_STICK_BUTTON_4 = new JoystickButton(r_stick, 4);
    RIGHT_STICK_BUTTON_4.whenHeld(new AimbotAndShoot());
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
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static SwerveControllerCommand swerveControllerCommand;

  public static Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    // set up trajectory config
    TrajectoryConfig config = new TrajectoryConfig(
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      3.0d
    ).setKinematics(DrivetrainSubsystem.s_kinematics);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    String trajectoryJSON = "paths/Unnamed.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (Exception e) {
      SmartDashboard.putString("was i able to find a file", "no you eedot");
    }

    swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      DrivetrainSubsystem::getPose,
      DrivetrainSubsystem.s_kinematics,
      xController,
      yController,
      thetaController,
      DrivetrainSubsystem::setModuleStates,
      s_drivetrainSubsystem
    );


    return new SequentialCommandGroup(
      new InstantCommand(() -> s_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> s_drivetrainSubsystem.stopModules())
      );
  }
}
