package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLightWrapper;

public class ShooterSubsystem extends SubsystemBase {
  public final WPI_TalonFX motor = new WPI_TalonFX(10, Constants.CANIVORE);
  public final WPI_TalonFX motor2 = new WPI_TalonFX(11, Constants.CANIVORE);
  
  public MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
  public ShooterSubsystem() {
    //motor2.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter fuckvelocty", motor.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void shoot(boolean shooting) {
    // if (speed == 0.0d){
    //   m_motorGroup.set(speed);
    // } else {
    //   m_motorGroup.set(speed / 2.0d + 0.5);
    // }
    
    SmartDashboard.putBoolean("speeeeed man", shooting);
    if (shooting != true){
      motor.set(ControlMode.PercentOutput, 0);
      motor2.set(ControlMode.PercentOutput, 0);
    } else {
      double speed = 0;
      if (LimeLightWrapper.hasTarget()){
        double y = LimeLightWrapper.getY();
        //speed = (-19.7654)*(y*y) + (281.237)*(y) + 14111.9;
        speed = (-80.9142)*(y) + 14041.2;
      } else {
        speed = 14800;
      }
      SmartDashboard.putNumber("adfhajdsfksajdhflkjsahfdkjasdf", speed);
      speed *= 5.5;
      
      motor.set(ControlMode.Velocity, speed);
      motor2.set(ControlMode.Velocity, -speed);
      // PIDController shooterController = new PIDController(0.6, 0.2, 0.0);
      // double shooterCommand = shooterController.calculate(motor.getSelectedSensorVelocity(), 0);
      // shooterController.close();
      // m_motorGroup.set(shooterCommand);
    }
  }
}
