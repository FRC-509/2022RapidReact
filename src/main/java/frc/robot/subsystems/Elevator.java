package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public final WPI_TalonFX elevatorMotor = new WPI_TalonFX(14, Constants.CANIVORE);
    private final WPI_TalonFX armMotor1 = new WPI_TalonFX(15, Constants.CANIVORE);
    private final WPI_TalonFX armMotor2 = new WPI_TalonFX(13, Constants.CANIVORE);

    public static final double ELEVATOR_FULLY_EXTENDED = 1000000; // fix this!

    public static final double ELEVATOR_FULLY_RETRACTED = -2200000; // fix this!

    public static final double ARM_POINTING_BACKWARDS = -2000000; // fix this!

    public static final double ARM_POINTING_FORWARDS = 6000000; // fix this!


    private static final double ArmPosHorizontal = 840;

    private static final double ArmTicksPerDegree = 4096 / 360;

    public MotorControllerGroup armGroup = new MotorControllerGroup(armMotor1, armMotor2);
    
    public Elevator() {
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        armMotor1.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        armMotor2.setInverted(true);
        elevatorMotor.setInverted(true);

        // armMotor1.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getElevatorPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void moveElevator(double input){
        double elevPos = elevatorMotor.getSelectedSensorPosition();
        elevatorMotor.set(softStop(-input, elevPos, ELEVATOR_FULLY_RETRACTED, ELEVATOR_FULLY_EXTENDED));
        SmartDashboard.putNumber("elevator pos:", elevPos);
    }

    public void fullyExtendElevator() {
        elevatorMotor.set(ControlMode.Position, ELEVATOR_FULLY_EXTENDED);
    }

    public void extendElevator(double val) {

        elevatorMotor.set(ControlMode.Position, ELEVATOR_FULLY_EXTENDED);
    }

    public void fullyRetractElevator() {
        elevatorMotor.set(ControlMode.Position, ELEVATOR_FULLY_RETRACTED);
    }

    public double getArmAngleDegrees() {
        double currentPos = (armMotor1.getSelectedSensorPosition() + armMotor2.getSelectedSensorPosition())/2.0d; // average sensor position between both motors
        return (currentPos - ArmPosHorizontal) / ArmTicksPerDegree;
        // return (armMotor1.getSelectedSensorPosition() + armMotor2.getSelectedSensorPosition())/2.0d + ARM_ANGLE_OFFSET;
    }

    public void setArmAngleDegrees(double deg) {
        armMotor1.set(ControlMode.Position, (deg + ArmPosHorizontal) * ArmTicksPerDegree);
        armMotor2.set(ControlMode.Position, (deg + ArmPosHorizontal) * ArmTicksPerDegree);
    }

    public void moveArm(double input){
        double armPos1 = armMotor1.getSelectedSensorPosition();
        armGroup.set(softStop((.25*(input)), armPos1, ARM_POINTING_BACKWARDS, ARM_POINTING_FORWARDS));
        SmartDashboard.putNumber("arm pos:", armPos1);
    }

    public double softStop(double input, double encoderPos, double low, double high){
        if (encoderPos < low){
            if (input < 0){
                input = 0;
            }
        }
        else if (encoderPos > high){
            if (input > 0){
                input = 0;
            }
        }
        return input;
    }
    
}
