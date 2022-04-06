package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(10, Constants.CANIVORE);
  private final WPI_TalonFX motor2 = new WPI_TalonFX(11, Constants.CANIVORE);
  
  private MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
  public ShooterSubsystem() {
    motor2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", motor.getSelectedSensorVelocity(motor.getDeviceID()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void shoot(double speed) {
    if (speed == 0.0d)
      m_motorGroup.set(speed);
    else
      m_motorGroup.set(speed / 2.0d + 0.5);
  }
}
