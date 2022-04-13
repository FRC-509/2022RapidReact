package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final static DoubleSolenoid m_intakeDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
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
    public static void intakeInit (){
        m_intakeDoublePCM.set(Value.kReverse);
    }
    public void upDown (){
        m_intakeDoublePCM.set(Value.kForward);
        DoubleSolenoid.Value val = m_intakeDoublePCM.get();

    }
    public void spin(double speed, DoubleSolenoid.Value intake) {
        if (speed == 0){
           
            motor.set(ControlMode.PercentOutput, 0.0d);
             m_intakeDoublePCM.set(intake);
        }
        else {
            //motor.set(ControlMode.PercentOutput, speed);
            //speed = speed / 2.0d;
            
            motor.set(ControlMode.PercentOutput, -speed);
            m_intakeDoublePCM.set(intake);
            // } else if (speed < -0.2){
            //     motor.set(ControlMode.PercentOutput, -speed);
            //     m_intakeDoublePCM.set(Value.kForward);
            // } else {
            //     motor.set(ControlMode.PercentOutput, 0.0d);
            // }
        }
    }
}
