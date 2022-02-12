package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    private final WPI_TalonFX theactualelevator = new WPI_TalonFX(14, "FastFD");
    private final WPI_TalonFX armot = new WPI_TalonFX(15, "FastFD");
    private final WPI_TalonFX armot2 = new WPI_TalonFX(13, "FastFD");

    private MotorControllerGroup leftGroup;
    
    public Elevator() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
