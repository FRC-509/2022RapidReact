
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
//import com.kauailabs.navx.frc.AHRS;
public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private MotorControllerGroup leftGroup;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;
  private MotorControllerGroup rightGroup;
  private DifferentialDrive drive;

  //DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21));
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  //PIDController leftPIDController = new PIDController(2.95, 0, 0);
  //PIDController rightPIDController = new PIDController(2.95, 0, 0);

  //AHRS gyro = new AHRS(SPI.Port.kMXP);
  public DriveTrain() {
    leftFront = new CANSparkMax(2, MotorType.kBrushed);
    //addChild("leftFront",(Sendable) leftFront);
    leftFront.setInverted(false);

    leftBack = new CANSparkMax(3, MotorType.kBrushed);
    //addChild("leftBack",(Sendable) leftBack);
    leftBack.setInverted(false);

    leftGroup = new MotorControllerGroup(leftFront, leftBack);
    addChild("leftGroup",leftGroup);


    rightFront = new CANSparkMax(5, MotorType.kBrushed);
    rightFront.setInverted(true);

    rightBack = new CANSparkMax(4, MotorType.kBrushed);
    rightBack.setInverted(true);

    rightGroup = new MotorControllerGroup(rightFront, rightBack);
    addChild("rightGroup",rightGroup);


    drive = new DifferentialDrive(leftGroup, rightGroup);
    addChild("Drive",drive);
    drive.setSafetyEnabled(true);
    drive.setExpiration(0.1);
    drive.setMaxOutput(1.0);
  }
  /*

  private Pose2d get2dHeading() {
    return null;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double left, double right) {
    drive.tankDrive(left, right);
  }}
