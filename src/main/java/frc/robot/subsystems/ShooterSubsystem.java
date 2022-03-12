
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(10);
  private final WPI_TalonFX motor2 = new WPI_TalonFX(11);
  
  private MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
  public ShooterSubsystem() {
    motor2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void shoot(double speed) {
    m_motorGroup.set(speed);
  }
}
