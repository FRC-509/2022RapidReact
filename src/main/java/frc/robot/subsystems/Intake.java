package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final WPI_TalonFX motor = new WPI_TalonFX(16, Constants.RIO_CANBUS);

    public Intake() {
        motor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    public void spin(double speed) {
        if (speed == 0)
            motor.set(ControlMode.PercentOutput, 0.0);
        else {
            speed = speed / 2.0d + 0.5d;
            if(speed > 0.2)
                motor.set(ControlMode.PercentOutput, -speed);
            else
                motor.set(ControlMode.PercentOutput, 0.0);
        }
    }
}


