package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final WPI_TalonFX motor = new WPI_TalonFX(100);
    private final WPI_TalonFX motor2 = new WPI_TalonFX(255);

    private MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);

    public Intake() {
        motor2.setInverted(true);
        motor.setInverted(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    public void spin(double speed){
        motor.set(ControlMode.Velocity, speed);
        motor2.set(ControlMode.Velocity, speed);
    }
}


