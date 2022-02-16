package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    public final WPI_TalonFX elevatorMotor = new WPI_TalonFX(14);
    private final WPI_TalonFX armMotor = new WPI_TalonFX(15);
    private final WPI_TalonFX armMotor2 = new WPI_TalonFX(13);

    public MotorControllerGroup armGroup = new MotorControllerGroup(armMotor, armMotor2);
    
    public Elevator() {
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
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
        elevatorMotor.set(input);

    }

    public void moveArm(double input){
        //commented until i look up the syntax for reversing a motor
        //armGroup.set(.1);
        armMotor.set(input);
        armMotor2.set(-input);
    }
    
}
