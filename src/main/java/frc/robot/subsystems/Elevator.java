package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public final WPI_TalonFX elevatorMotor = new WPI_TalonFX(14, Constants.CANIVORE);
    public final WPI_TalonFX armMotor1 = new WPI_TalonFX(15, Constants.CANIVORE);
    public final WPI_TalonFX armMotor2 = new WPI_TalonFX(13, Constants.CANIVORE);

    public MotorControllerGroup armGroup = new MotorControllerGroup(armMotor1, armMotor2);
    
    public Elevator() {
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        armMotor1.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        armMotor2.setInverted(true);
        elevatorMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
    public void moveElevator(double input){
        double elevPos = elevatorMotor.getSelectedSensorPosition();
        elevatorMotor.set(softStop(-input, elevPos, -2200000, 1000000));
        SmartDashboard.putNumber("elevator pos:", elevPos);
    }

    public void moveArm(double input){
        double armPos1 = armMotor1.getSelectedSensorPosition();
        armGroup.set(softStop((.25*(input)), armPos1, -2000000, 6000000));
        SmartDashboard.putNumber("arm pos:", armPos1);
    }

    public void moveElevatorPos(double input){
        elevatorMotor.set(ControlMode.Position, input);
    }

    public void moveArmPos(double input){
        armMotor1.set(ControlMode.Position, input);
        armMotor2.set(ControlMode.Position, input);
    }

    public double softStop(double input, double encoderPos, int low, int high){
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