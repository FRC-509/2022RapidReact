package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SDSConstants;

public class SwerveDrive extends SubsystemBase {
	public static final PIDController xController = new PIDController(1.5, 0, 0);
  	public static final PIDController yController = new PIDController(1.5, 0, 0);
	
	private final Field2d m_field = new Field2d();
	public final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(
		// Front left
		new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Front right
		new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back left
		new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back right
		new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
	);
	private SwerveModule m_backLeft;
	private SwerveModule m_backRight;
	private SwerveModule m_frontLeft;
	private SwerveModule m_frontRight;
	private double m_desiredAngle_deg;
	private double[] m_lastYpr = {0, 0, 0};
	private WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);
	private Pose2d m_pose;
	private ChassisSpeeds m_speeds = new ChassisSpeeds(0, 0, 0);
	private SwerveDriveOdometry m_swerveOdometry;
	private double m_angleSetpointVal;
	private double m_angleErrorVal;

	public SwerveDrive() {
		m_backLeft = new SwerveModule(Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET, Constants.RIO_CANBUS, Constants.RIO_CANBUS);
		m_backRight = new SwerveModule(Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET, Constants.CANIVORE, Constants.CANIVORE);
		m_frontLeft = new SwerveModule(Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET, Constants.RIO_CANBUS, Constants.RIO_CANBUS);
		m_frontRight = new SwerveModule(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET, Constants.CANIVORE, Constants.CANIVORE);
		m_pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
		m_swerveOdometry = new SwerveDriveOdometry(m_swerveKinematics, Rotation2d.fromDegrees(getYaw()), m_pose);
	}

	public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldCentric) {
		if (fieldCentric)
			m_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity_m_per_s, yVelocity_m_per_s, omega_rad_per_s, m_pose.getRotation());
		else
			m_speeds = new ChassisSpeeds(xVelocity_m_per_s, yVelocity_m_per_s, (omega_rad_per_s));
		SwerveModuleState[] moduleStates = m_swerveKinematics.toSwerveModuleStates(m_speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SDSConstants.MAX_VELOCITY_METERS_PER_SECOND);
		SwerveModuleState frontLeft = moduleStates[0];
		SwerveModuleState frontRight = moduleStates[1];
		SwerveModuleState backLeft = moduleStates[2];
		SwerveModuleState backRight = moduleStates[3];
		m_frontLeft.setDesiredState(frontLeft);
		m_frontRight.setDesiredState(frontRight);
		m_backLeft.setDesiredState(backLeft);
		m_backRight.setDesiredState(backRight);
	}

	public ChassisSpeeds getDesiredSpeeds() {
		return m_speeds;
	}

	public SwerveDriveKinematics getKinematics() {
		return m_swerveKinematics;
	}

	public void resetYawAngle() {
		m_pigeon.setYaw(0);
	}

	public void populateAngleControl(double angleError, double setpoint) {
		m_angleErrorVal = angleError;
		m_angleSetpointVal = setpoint;
	}

	public Pose2d getPose() {
		return m_swerveOdometry.getPoseMeters();
	}

	public void setPose(Pose2d desiredPose) {
		m_swerveOdometry.resetPosition(desiredPose, Rotation2d.fromDegrees(getYaw()));
	}

	public void setAllModulesToAngle(double angle_deg) {
		m_backLeft.setToAngle(angle_deg);
		m_backRight.setToAngle(angle_deg);
	}

	public void resetAllModulesIntegrated() {
		m_backLeft.resetIntegratedEncoder();
		m_backRight.resetIntegratedEncoder();
		m_frontLeft.resetIntegratedEncoder();
		m_frontRight.resetIntegratedEncoder();
	}

	public void stopAllModules() {
		m_backLeft.stop();
		m_backRight.stop();
		m_frontLeft.stop();
		m_frontRight.stop();
	}

	private double getYaw() {
		return m_pigeon.getYaw();
	}

	public double getPitchDeg() {
		return m_lastYpr[1];
	}

	public double getRollDeg() {
		return m_lastYpr[2];
	}

	@Override
	public void periodic() {
		m_field.setRobotPose(m_swerveOdometry.getPoseMeters());
		var gyroAngle = Rotation2d.fromDegrees(getYaw());
		m_pose = m_swerveOdometry.update(gyroAngle, m_frontLeft.getSwerveState(), m_frontRight.getSwerveState(), m_backLeft.getSwerveState(), m_backRight.getSwerveState());
		// resetAllModulesIntegrated();
		m_pigeon.getYawPitchRoll(m_lastYpr);
	}

	public void showTrajectory(Trajectory trajectory) {
		m_field.getObject("traj").setTrajectory(trajectory);
	}

	public void resetPose() {
		// resetYawAngle();
		m_swerveOdometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(getYaw()));
	}

	public void wheelsXFormation() {
		m_frontLeft.setToAngle(45);
		m_frontRight.setToAngle(135);
		m_backLeft.setToAngle(135);
		m_backRight.setToAngle(45);
	}

	public void setModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, SDSConstants.MAX_VELOCITY_METERS_PER_SECOND);
		SwerveModuleState frontLeft = states[0];
		SwerveModuleState frontRight = states[1];
		SwerveModuleState backLeft = states[2];
		SwerveModuleState backRight = states[3];
		// Controller velocity axis is backwards
		// frontLeft.speedMetersPerSecond *= -1;
		// frontRight.speedMetersPerSecond *= -1;
		// backLeft.speedMetersPerSecond *= -1;
		// backRight.speedMetersPerSecond *= -1;

		m_frontLeft.setDesiredState(frontLeft);
		m_frontRight.setDesiredState(frontRight);
		m_backLeft.setDesiredState(backLeft);
		m_backRight.setDesiredState(backRight);
	}
}
