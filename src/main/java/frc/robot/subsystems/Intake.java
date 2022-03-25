package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid m_intakeDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    private final TalonFX motor = new TalonFX(16, Constants.RIO_CANBUS);

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

    public void spin(double speed, DoubleSolenoid.Value intake) {
        if (speed == 0)
            motor.set(ControlMode.PercentOutput, 0.0d);
        else {
            speed = speed / 2.0d + 0.5d;
            if(speed > 0.2) {
                m_intakeDoublePCM.set(intake);
                motor.set(ControlMode.PercentOutput, -speed);
            }
            else {
                m_intakeDoublePCM.set(Value.kReverse);
                motor.set(ControlMode.PercentOutput, 0.0d);
            }
        }
    }
}
