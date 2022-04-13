
package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
<<<<<<< HEAD
import frc.robot.autonomous.SixBallAuto;
import frc.robot.autonomous.five_ball;
import frc.robot.autonomous.three_ball;
import frc.robot.autonomous.two_ball;
import frc.robot.autonomous.new_five_ball;

=======
import frc.robot.autonomous.five_ball;
import frc.robot.autonomous.three_ball;
import frc.robot.autonomous.two_ball;
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.commands.AimBot;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  public static final Indexer s_indexer = new Indexer();
  public static final Intake s_intake = new Intake();
  public static final Elevator s_elevator = new Elevator();
  public static final DriveTrainSubsystem s_drivetrainSubsystem = new DriveTrainSubsystem();

  public static final Joystick l_stick = new Joystick(1);
  public static final Joystick r_stick = new Joystick(0);

  public static final GenericHID s_logiController = new GenericHID(2);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private static Trajectory trajectory = null;
  //public static final CameraServer camera = new CameraServer();
  /*
  Code for auto aiming during teleop, james if you are daring enough you can try replacing
  the last lambda in s_drivetrainSubsystem.setDefaultCommand with a call to this function.
  */
  public static double angularDeltaCalculator() {
<<<<<<< HEAD
    if (r_stick.getRawButton(1) && LimeLightWrapper.hasTarget())
      return .75*Math.toRadians(LimeLightWrapper.getX() - (20*l_stick.getX())) * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    else
      return modifyAxis(r_stick.getX()) * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }
  public static UsbCamera cam = new UsbCamera("509cam", 1);
  //public static UsbCamera cam2 = new UsbCamera("509cam2", 1);
=======
    if (l_stick.getRawButton(3))
      return -0.01 * LimeLightWrapper.getX() * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    else
      return modifyAxis(r_stick.getX()) * DriveTrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }
  public static UsbCamera cam = new UsbCamera("509cam", 0);
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_chooser.setDefaultOption("2-ball auto, use this", new two_ball());
    m_chooser.addOption("3-ball auto, use this if you dare", new three_ball());
    m_chooser.addOption("5-ball auto, DO NOT USE unless THERE IS NO OTHER OPTION", new five_ball());
    m_chooser.addOption("bruh moment no use", new new_five_ball());
    m_chooser.addOption("paff weaveww owo", new SixBallAuto());
    SmartDashboard.putData(m_chooser);
    // Configure the button bindings
    configureButtonBindings();

    s_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      () -> modifyAxis(l_stick.getY()) * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(l_stick.getX()) * DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> angularDeltaCalculator()
    ));

    s_elevator.setDefaultCommand(new ElevatorCommand(
      () -> s_logiController.getRawAxis(1),
      () -> -s_logiController.getRawAxis(5)
    ));
    
<<<<<<< HEAD
    // m_chooser.setDefaultOption("2-ball auto, use this", new two_ball());
    // m_chooser.addOption("3-ball auto, use this if you dare", new three_ball());
    // m_chooser.addOption("5-ball auto, DO NOT USE unless THERE IS NO OTHER OPTION", new five_ball());
    // m_chooser.addOption("test auto ", new testAuto());
    // m_chooser.addOption("paff weaveww owo", new SixBallAuto());
=======
    m_chooser.setDefaultOption("2-ball auto, use this", new two_ball());
    m_chooser.addOption("3-ball auto, use this if you dare", new three_ball());
    m_chooser.addOption("5-ball auto, DO NOT USE unless THERE IS NO OTHER OPTION", new five_ball());
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7

    // SmartDashboard.putData("Command Chooser", m_chooser);
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
<<<<<<< HEAD
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(() -> true));
    
=======
    RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(() -> 0.32));
    SmartDashboard.putNumber("shooter fuckvelocty", s_shooterSubsystem.motor.getSelectedSensorVelocity(10));
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
    //RIGHT_STICK_BUTTON_1.whenHeld(new ShooterCommand(() -> ((l_stick.getRawAxis(3) + 1.0d)/2.0)));
    // Button 1 on the right stick must be held down to shoot.
    SmartDashboard.putNumber("bruh value two", ((l_stick.getRawAxis(3)+1.0)/2.0));
    JoystickButton LEFT_STICK_BUTTON_2 = new JoystickButton(l_stick, 2);
    // LEFT_STICK_BUTTON_2.whenPressed(new IntakeDown());
    LEFT_STICK_BUTTON_2.whenHeld(new IntakeSpit());
    LEFT_STICK_BUTTON_2.whenReleased(new IntakeUp());
    JoystickButton LEFT_STICK_BUTTON_1 = new JoystickButton(l_stick, 1);
    // LEFT_STICK_BUTTON_1.whenPressed(new IntakeDown());
    LEFT_STICK_BUTTON_1.whenHeld(new IntakeSpin());
    LEFT_STICK_BUTTON_1.whenReleased(new IntakeUp());

    JoystickButton LEFT_STICK_BUTTON_11 = new JoystickButton(l_stick, 11);
    LEFT_STICK_BUTTON_11.whenPressed(new InstantCommand(s_drivetrainSubsystem::zeroGyroscope, s_drivetrainSubsystem));
    
    JoystickButton LEFT_STICK_BUTTON_4 = new JoystickButton(l_stick, 4);
<<<<<<< HEAD
    LEFT_STICK_BUTTON_4.whenPressed(new InstantCommand(s_drivetrainSubsystem::zeroGyroscope, s_drivetrainSubsystem));
    //RIGHT_STICK_BUTTON_1.whenHeld(new AimBot());
=======
    LEFT_STICK_BUTTON_4.whenHeld(new AimBot());
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7

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
  
  public Command getAutonomousCommand() {
<<<<<<< HEAD
=======
    
    TrajectoryBuilderWrapper wrapper = new TrajectoryBuilderWrapper("paths/rishabruh.wpilib.json");
    SwerveControllerCommand swerveControllerCommand = wrapper.getSwerveControllerCommand();
    
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> s_drivetrainSubsystem.resetOdometry(wrapper.getTrajectory().getInitialPose())),
    //   swerveControllerCommand,
    //   new InstantCommand(() -> s_drivetrainSubsystem.stopModules()),
    //   new AimBot(),
    //   new ShooterCommand(() -> 0.35).withTimeout(2.0d)
    // );
<<<<<<< HEAD
    return m_chooser.getSelected();
    //return new SixBallAuto();
=======
    
    return m_chooser.getSelected();
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
    
    // return new SequentialCommandGroup(
    //   // //drive forwards + succ      
    //   // new ParallelCommandGroup(
    //   //   //Drive forwards off tarmac
    //   //   new DefaultDriveCommand(() -> -0.4, () -> 0, 0.0d).withTimeout(1),
    //   //   //succ 1
    //   //   new SequentialCommandGroup(IntakeDown(), IntakeSpin().withTimeout(0.5), IntakeUp())
    //   // ),

    //   // //turn and shoot
    //   // new
      
    // );
  }
  
}
