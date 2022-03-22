package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.SDSConstants;

public class SwerveModule {
	private TalonFX m_turningMotor;
	private TalonFX m_driveMotor;
	private DutyCycleEncoder m_encoder;
	private PIDController m_rotationController;
	private SwerveModuleState m_swerveModuleState = new SwerveModuleState();
	private double m_encoderOffset;
	private SwerveModuleState m_desiredState = new SwerveModuleState();
	private ErrorCode m_turnMotorErrorCode;

	public SwerveModule(int turnMotorCanID, int driveMotorCanID, int encoderPort, double encoderOffset) {
		this(turnMotorCanID, driveMotorCanID, encoderPort, encoderOffset, "rio", "rio");
	}

	public SwerveModule(int turnMotorCanID, int driveMotorCanID, int encoderPort, double encoderOffset, String turnMotorCanBus, String driveMotorCanBus) {
		m_turningMotor = new TalonFX(turnMotorCanID, turnMotorCanBus);
		m_driveMotor = new TalonFX(driveMotorCanID, driveMotorCanBus);
		m_encoder = new DutyCycleEncoder(encoderPort);
		m_turningMotor.configFactoryDefault();
		m_driveMotor.configFactoryDefault();
		m_rotationController = new PIDController(3.0d, 0, 0);
		// m_rotationController = new PIDController(Constants.kTurningMotorKp, Constants.kTurningMotorKi, Constants.kTurningMotorKd);
		m_rotationController.enableContinuousInput(-180, 180);
		m_encoder.setDistancePerRotation(360);
		m_swerveModuleState.speedMetersPerSecond = 0;
		m_swerveModuleState.angle = Rotation2d.fromDegrees(m_encoder.getDistance());
		m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		m_encoderOffset = encoderOffset;
		m_driveMotor.setNeutralMode(NeutralMode.Brake);
		m_driveMotor.enableVoltageCompensation(true);
		m_driveMotor.configVoltageCompSaturation(12);
		m_driveMotor.setNeutralMode(NeutralMode.Brake);
		m_driveMotor.setSelectedSensorPosition(0);
		m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 3, 1));
		m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 3, 1));
		m_driveMotor.config_kP(0, 0.001);
		m_driveMotor.config_kI(0, 0.001);
		m_driveMotor.config_kD(0, 0.001);
		m_driveMotor.config_kF(0, 0.001);

		m_turningMotor.setNeutralMode(NeutralMode.Brake);
		m_turningMotor.enableVoltageCompensation(true);
		m_turningMotor.configVoltageCompSaturation(12);
		m_turningMotor.setNeutralMode(NeutralMode.Brake);
		m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 3, 1));
		m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 3, 1));
		m_turningMotor.config_kP(0, 3.0);
		m_turningMotor.config_kI(0, 0.0);
		m_turningMotor.config_kD(0, 0.0);
		// m_turningMotor.setSelectedSensorPosition(degreesToFalcon(getEncoderValue(), Constants.kTurnGearRatio));
		resetIntegratedEncoder();
	}

	public static double normalizedDeg(double deg) {
		//  if (deg > -180 && deg < 180){
		//      return deg;
		//  }
		// return Rotation2d.fromDegrees(deg).rotateBy(new Rotation2d()).getDegrees();

		double angleDegMod = deg % 360.0;
		if (angleDegMod < 0.0) {
			angleDegMod += 360.0;
		}
		return angleDegMod;
	}

	public static double tp100ms2mps(double tp100ms) {
		double tpMs = tp100ms / 100;
		double tps = tpMs * 1000;
		double rps_motor = tps / 2048;
		double rps_wheel = rps_motor * SDSConstants.MK4_L1_DRIVE_REDUCTION;
		return rps_wheel * SDSConstants.MK4_L1_WHEEL_CIRCUMFERENCE;
	}

	public static double mps2ticksPer100ms(double mps) {
		double rps_wheel = mps / SDSConstants.MK4_L1_WHEEL_CIRCUMFERENCE;
		double rps_motor = rps_wheel * SDSConstants.MK4_L1_GEAR_RATIO;
		double tps = rps_motor * 2048;
		double tpMs = tps / 1000;
		return tpMs * 100;
	}

	public static double degreesToFalcon(double degrees, double gearRatio) {
		return degrees / (360 / (gearRatio * 2048.0));
	}

	public static double falconToDegrees(double falconTicks, double gearRatio) {
		return falconTicks * (360 / (gearRatio * 2048.0));
	}

	public static double wrapToScope(double currentAngleDeg, double referenceAngleDeg) {
		double currentAngleDegMod = currentAngleDeg % (360.0);
		if (currentAngleDegMod < 0.0) {
			currentAngleDegMod += 360.0;
		}

		// The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
		double adjustedReferenceAngleDeg = referenceAngleDeg + currentAngleDeg - currentAngleDegMod;
		if (referenceAngleDeg - currentAngleDegMod > 180.0) {
			adjustedReferenceAngleDeg -= 360.0;
		} else if (referenceAngleDeg - currentAngleDegMod < -180.0) {
			adjustedReferenceAngleDeg += 360.0;
		}
		return adjustedReferenceAngleDeg;
	}

	public void resetIntegratedEncoder() {
		m_turnMotorErrorCode = m_turningMotor.setSelectedSensorPosition(degreesToFalcon(getEncoderValue(), SDSConstants.MK4_L1_GEAR_RATIO));
	}

	public void setToAngle(double angle_deg) {
		m_desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(angle_deg));
		setAngleMotor();
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		m_desiredState = SwerveModuleState.optimize(desiredState, getSwerveState().angle);
		m_rotationController.setSetpoint(normalizedDeg(m_desiredState.angle.getDegrees()));

		if (Math.abs(m_desiredState.speedMetersPerSecond) > 0.015) {
			setAngleMotor();
		}
		m_driveMotor.set(ControlMode.Velocity, mps2ticksPer100ms(m_desiredState.speedMetersPerSecond));
	}

	private void setAngleMotor() {
		if (SDSConstants.USE_INTEGRATED_PID) {
			double angleSetpointDeg = wrapToScope(m_swerveModuleState.angle.getDegrees(), m_desiredState.angle.getDegrees());
			m_turningMotor.set(ControlMode.Position, degreesToFalcon(angleSetpointDeg, SDSConstants.MK4_L1_GEAR_RATIO));
		} else {
			m_turningMotor.set(ControlMode.PercentOutput, m_rotationController.calculate(getEncoderValue()));
		}
	}

	public SwerveModuleState getDesiredSwerveState() {
		return m_desiredState;
	}

	public SwerveModuleState getSwerveState() {
		if (RobotBase.isSimulation()) {
			return m_desiredState; // Pass through desired state.
		}
		m_swerveModuleState.speedMetersPerSecond = tp100ms2mps(m_driveMotor.getSelectedSensorVelocity());
		m_swerveModuleState.angle = Rotation2d.fromDegrees(falconToDegrees(m_turningMotor.getSelectedSensorPosition(), SDSConstants.MK4_L1_GEAR_RATIO));
		return m_swerveModuleState;
	}

	double getEncoderValue() {
		return normalizedDeg(m_encoder.getDistance()) - m_encoderOffset;
	}

	double getFalconEncoderEntry() {
		return normalizedDeg(falconToDegrees(m_turningMotor.getSelectedSensorPosition(), SDSConstants.MK4_L1_GEAR_RATIO));
	}

	public void stop() {
		m_turningMotor.set(ControlMode.PercentOutput, 0);
		m_driveMotor.set(ControlMode.PercentOutput, 0);
	}
	
}
