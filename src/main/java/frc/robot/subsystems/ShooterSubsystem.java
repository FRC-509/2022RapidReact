package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public final WPI_TalonFX motor = new WPI_TalonFX(10, Constants.CANIVORE);
  public final WPI_TalonFX motor2 = new WPI_TalonFX(11, Constants.CANIVORE);
  
  public MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
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
    if (speed == 0.0d){
      m_motorGroup.set(speed);
    } else {
      m_motorGroup.set(speed / 2.0d + 0.5);
    }

    // if (speed == 0.0d){
    //   motor.set(ControlMode.Velocity, speed);
    //   motor2.set(ControlMode.Velocity, speed);
    // } else {
    //   motor.set(ControlMode.Velocity, speed / 2.0d + 0.5);
    //   motor2.set(ControlMode.Velocity, speed / 2.0d + 0.5);
    // }
  }
}
